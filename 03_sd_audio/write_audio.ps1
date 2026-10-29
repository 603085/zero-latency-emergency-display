# =============================================================================
#  write_audio.ps1  --  把 audio_sd.bin 写到 SD 卡第 131072 扇区
# =============================================================================
#  用法（在【管理员】PowerShell 里跑）：
#
#      # 1) 先检查安全性（只读，不需要管理员也行）
#      powershell -NoProfile -ExecutionPolicy Bypass -File write_audio.ps1 -CheckOnly
#
#      # 2) 备份 + 写入 + 校验
#      powershell -NoProfile -ExecutionPolicy Bypass -File write_audio.ps1
#
#      # 3) 只校验（卡上应该已经有音频了）
#      powershell -NoProfile -ExecutionPolicy Bypass -File write_audio.ps1 -VerifyOnly
#
#  ---------------------------------------------------------------------------
#  为什么写可移动卷需要特殊手段（踩过的坑，别再走弯路）
#  ---------------------------------------------------------------------------
#  * Set-Disk -IsOffline 对 U 盘/SD 卡报：
#        Reversible media cannot be set to offline
#    可移动介质**不允许脱机**，这是 Windows 的硬限制。
#  * 正确途径是卷锁定：
#        CreateFile("\\.\E:") -> FSCTL_LOCK_VOLUME -> FSCTL_DISMOUNT_VOLUME
#        写 -> FSCTL_UNLOCK_VOLUME -> 关闭
#  * 而裸写卷必须有管理员权限（UAC 提升）。
#    DSH/普通进程是 Medium 完整性，写不了；所以这一步必须人工执行。
#
#  ---------------------------------------------------------------------------
#  数据布局（由 RTL 常量固定，别随便改）
#  ---------------------------------------------------------------------------
#    audio_sd_loader.v : ALARM_BASE_SECTOR = 131072, 471 扇区
#                        RING_BASE_SECTOR  = 131543, 249 扇区
#    合计 720 扇区 = 368640 字节 = 360 KB
#    扇区 131072 -> 字节偏移 67108864 (64 MB)
#
#  ---------------------------------------------------------------------------
#  本脚本的流程
#  ---------------------------------------------------------------------------
#    [0] 自检（管理员 / 音频文件 / 磁盘识别）
#    [1] 安全检查：目标扇区是否落在现有文件之外（用文件系统驱动查真实 LCN）
#    [2] 备份目标区域（1 MB，留后路）
#    [3] 卷锁定 -> 写 -> 解锁
#    [4] 读回逐字节比对 + SHA256
#    [5] 确认 E:\ 下文件数没变少
# =============================================================================
[CmdletBinding()]
param(
    [int]$DiskNumber = -1,
    [string]$DriveLetter = '',
    [string]$BinPath = '',
    [switch]$Yes,
    [switch]$CheckOnly,
    [switch]$VerifyOnly
)

$ErrorActionPreference = 'Continue'
$SECTOR        = 512
$TARGET_SECTOR = 131072
$OFFSET        = [int64]$TARGET_SECTOR * $SECTOR           # 67108864
$EXPECT_SHA256 = '05A5995EC59CE13ED049E32B1233B7C50D23912E9814FB011326C6EA113C6BE4'
$BACKUP_SECTORS = 2048

function Say($m, $c = 'Gray') { Write-Host $m -ForegroundColor $c }
function Head($m) { Write-Host ''; Write-Host "--- $m ---" -ForegroundColor Cyan }

# ---------------------------------------------------------------- 找音频文件
if (-not $BinPath) {
    $cands = @(
        'D:\TaskC\work_v2\sd_audio\audio_sd.bin',
        'D:\202611嵌赛\TaskC交付_音频方向\sd_audio\audio_sd.bin',
        'D:\202611嵌赛\sounds\audio_sd.bin',
        (Join-Path $PSScriptRoot 'audio_sd.bin')
    )
    foreach ($c in $cands) { if (Test-Path -LiteralPath $c) { $BinPath = $c; break } }
}
if (-not $BinPath -or -not (Test-Path -LiteralPath $BinPath)) {
    Say '!! 找不到 audio_sd.bin' 'Red'
    Say '   用 -BinPath "完整路径" 指定' 'Yellow'
    exit 1
}
$BIN = $BinPath
$BACKUP = Join-Path $PSScriptRoot 'sd_region_backup.bin'

# ---------------------------------------------------------------- 原生 API
$cs = @'
using System;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

public static class RawVol {
    [StructLayout(LayoutKind.Sequential)]
    public struct SVCN { public long StartingVcn; }

    [DllImport("kernel32.dll", SetLastError=true, CharSet=CharSet.Unicode)]
    static extern SafeFileHandle CreateFileW(string name, uint access, uint share,
        IntPtr sec, uint disp, uint flags, IntPtr tmpl);

    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool DeviceIoControl(SafeFileHandle h, uint code,
        ref SVCN inBuf, uint inSize, IntPtr outBuf, uint outSize, out uint ret, IntPtr ov);
    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool DeviceIoControl(SafeFileHandle h, uint code,
        IntPtr inBuf, uint inSize, IntPtr outBuf, uint outSize, out uint ret, IntPtr ov);

    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool SetFilePointerEx(SafeFileHandle h, long dist, out long np, uint method);

    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool WriteFile(SafeFileHandle h, byte[] buf, uint n, out uint written, IntPtr ov);

    [DllImport("kernel32.dll", SetLastError=true)]
    static extern bool ReadFile(SafeFileHandle h, byte[] buf, uint n, out uint read, IntPtr ov);

    const uint GENERIC_READ  = 0x80000000;
    const uint GENERIC_WRITE = 0x40000000;
    const uint SHARE_RW      = 3;
    const uint OPEN_EXISTING = 3;
    const uint WRITE_THROUGH = 0x80000000;
    const uint LOCK_VOLUME     = 0x00090018;
    const uint UNLOCK_VOLUME   = 0x0009001C;
    const uint DISMOUNT_VOLUME = 0x00090020;
    const uint GET_RETRIEVAL   = 0x00090073;
    const uint FILE_BEGIN      = 0;
    const char SEP = '|';

    static string LockIt(SafeFileHandle h) {
        uint d;
        if (!DeviceIoControl(h, LOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero)) {
            int e1 = Marshal.GetLastWin32Error();
            DeviceIoControl(h, DISMOUNT_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero);
            if (!DeviceIoControl(h, LOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero)) {
                int e2 = Marshal.GetLastWin32Error();
                return "ERRLOCK" + SEP + e1 + SEP + e2;
            }
        }
        DeviceIoControl(h, DISMOUNT_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero);
        return "OK";
    }

    // 只读：读特定偏移，返回 "OK|路径|字节数" 或 "ERR..."
    public static string DumpAt(string vol, long offset, int length, string outPath) {
        var h = CreateFileW(vol, GENERIC_READ, SHARE_RW, IntPtr.Zero, OPEN_EXISTING, 0, IntPtr.Zero);
        if (h.IsInvalid) { int e = Marshal.GetLastWin32Error(); h.Close(); return "ERRopen" + SEP + e; }
        try {
            long np;
            if (!SetFilePointerEx(h, offset, out np, FILE_BEGIN)) {
                int e = Marshal.GetLastWin32Error(); return "ERRseek" + SEP + e;
            }
            byte[] all = new byte[length];
            int pos = 0;
            while (pos < length) {
                int n = Math.Min(131072, length - pos);
                byte[] b = new byte[n];
                uint got;
                if (!ReadFile(h, b, (uint)n, out got, IntPtr.Zero)) {
                    int e = Marshal.GetLastWin32Error(); return "ERRread" + SEP + pos + SEP + e;
                }
                if (got == 0) break;
                Array.Copy(b, 0, all, pos, (int)got);
                pos += (int)got;
            }
            System.IO.File.WriteAllBytes(outPath, all);
            return "OK" + SEP + outPath + SEP + pos;
        } finally { h.Close(); }
    }

    // 只在内存里比，避免大数组跨 PowerShell 边界
    public static string CompareAt(string vol, long offset, byte[] src) {
        var h = CreateFileW(vol, GENERIC_READ, SHARE_RW, IntPtr.Zero, OPEN_EXISTING, 0, IntPtr.Zero);
        if (h.IsInvalid) { int e = Marshal.GetLastWin32Error(); h.Close(); return "ERRopen" + SEP + e; }
        try {
            long np;
            if (!SetFilePointerEx(h, offset, out np, FILE_BEGIN)) {
                int e = Marshal.GetLastWin32Error(); return "ERRseek" + SEP + e;
            }
            const int CHUNK = 131072;
            int pos = 0, ndiff = 0, firstDiff = -1;
            byte[] buf = new byte[CHUNK];
            while (pos < src.Length) {
                int n = Math.Min(CHUNK, src.Length - pos);
                uint got;
                if (!ReadFile(h, buf, (uint)n, out got, IntPtr.Zero)) {
                    int e = Marshal.GetLastWin32Error(); return "ERRread" + SEP + pos + SEP + e;
                }
                if (got == 0) return "ERREOF" + SEP + pos;
                for (int i = 0; i < got; i++) {
                    if (buf[i] != src[pos + i]) {
                        ndiff++;
                        if (firstDiff < 0) firstDiff = pos + i;
                    }
                }
                pos += (int)got;
            }
            return "OK" + SEP + ndiff + SEP + firstDiff + SEP + pos;
        } finally { h.Close(); }
    }

    // 写：锁卷 -> 写 -> 解锁
    public static string Write(string vol, long offset, byte[] data) {
        var h = CreateFileW(vol, GENERIC_READ | GENERIC_WRITE, SHARE_RW,
            IntPtr.Zero, OPEN_EXISTING, WRITE_THROUGH, IntPtr.Zero);
        if (h.IsInvalid) { int e = Marshal.GetLastWin32Error(); h.Close(); return "ERRopen" + SEP + e; }
        try {
            string lk = LockIt(h);
            if (lk != "OK") return lk;
            long np;
            if (!SetFilePointerEx(h, offset, out np, FILE_BEGIN)) {
                int e = Marshal.GetLastWin32Error();
                uint d; DeviceIoControl(h, UNLOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero);
                return "ERRseek" + SEP + e;
            }
            const int CHUNK = 262144;
            int done = 0;
            while (done < data.Length) {
                int n = Math.Min(CHUNK, data.Length - done);
                byte[] part = new byte[n];
                Array.Copy(data, done, part, 0, n);
                uint wrote;
                if (!WriteFile(h, part, (uint)n, out wrote, IntPtr.Zero)) {
                    int e = Marshal.GetLastWin32Error();
                    uint d; DeviceIoControl(h, UNLOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero);
                    return "ERRwrite" + SEP + done + SEP + e;
                }
                if (wrote != n) {
                    uint d; DeviceIoControl(h, UNLOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out d, IntPtr.Zero);
                    return "ERRSHORT" + SEP + done + SEP + wrote;
                }
                done += n;
            }
            uint dd; DeviceIoControl(h, UNLOCK_VOLUME, IntPtr.Zero, 0, IntPtr.Zero, 0, out dd, IntPtr.Zero);
            return "OK" + SEP + done;
        } finally { h.Close(); }
    }

    // 查一个文件的真实 LCN 范围，返回 "firstLcn|lastLcn|err"
    public static string Extent(string path) {
        var h = CreateFileW(path, GENERIC_READ, SHARE_RW, IntPtr.Zero, OPEN_EXISTING, 0, IntPtr.Zero);
        if (h.IsInvalid) { h.Close(); return "-1|-1|" + Marshal.GetLastWin32Error(); }
        try {
            var inb = new SVCN { StartingVcn = 0 };
            uint outSize = 65536;
            IntPtr ob = Marshal.AllocHGlobal((int)outSize);
            try {
                uint ret;
                if (!DeviceIoControl(h, GET_RETRIEVAL, ref inb, (uint)Marshal.SizeOf(inb),
                        ob, outSize, out ret, IntPtr.Zero)) {
                    return "-1|-1|" + Marshal.GetLastWin32Error();
                }
                uint count = (uint)Marshal.ReadInt32(ob, 0);
                long startVcn = Marshal.ReadInt64(ob, 8);
                long first = -1, lastEnd = -1;
                for (int i = 0; i < count && i < 256; i++) {
                    int off = 16 + i * 16;
                    long nextVcn = Marshal.ReadInt64(ob, off);
                    long lcn     = Marshal.ReadInt64(ob, off + 8);
                    long prevVcn = (i == 0) ? startVcn : Marshal.ReadInt64(ob, off - 16);
                    if (lcn < 0) continue;
                    if (first < 0) first = lcn;
                    long run = nextVcn - prevVcn;
                    long e = lcn + run - 1;
                    if (e > lastEnd) lastEnd = e;
                }
                return first + "|" + lastEnd + "|0";
            } finally { Marshal.FreeHGlobal(ob); }
        } finally { h.Close(); }
    }
}
'@
Add-Type -TypeDefinition $cs -Language CSharp | Out-Null

# ============================================================ [0] 自检
$isAdmin = (New-Object Security.Principal.WindowsPrincipal(
    [Security.Principal.WindowsIdentity]::GetCurrent())).IsInRole(
    [Security.Principal.WindowsBuiltInRole]::Administrator)

$bytes = [System.IO.File]::ReadAllBytes($BIN)
$shaSrc = (Get-FileHash -LiteralPath $BIN -Algorithm SHA256).Hash

Say ("音频文件 : {0}" -f $BIN)
Say ("大小     : {0} 字节 = {1} 扇区" -f $bytes.Length, ($bytes.Length / $SECTOR))
Say ("SHA256   : {0}" -f $shaSrc)
if ($shaSrc -ne $EXPECT_SHA256) {
    Say ("预期     : {0}" -f $EXPECT_SHA256) 'Yellow'
    Say '!! 哈希和已知版本不一致 —— 如果是有意换的音频可以忽略，否则先确认文件没坏。' 'Yellow'
}
Say ("管理员   : {0}" -f $isAdmin) $(if ($isAdmin) { 'Green' } else { 'Yellow' })

# 找盘
if ($DiskNumber -lt 0) {
    $cands = Get-Disk | Where-Object { $_.BusType -eq 'USB' -and $_.Number -ne 0 }
    if (-not $cands) { Say '!! 没找到 USB 盘，用 -DiskNumber N 指定' 'Red'; exit 1 }
    if (@($cands).Count -gt 1) {
        Say '找到多块 USB 盘，请用 -DiskNumber 指定：' 'Red'
        $cands | Format-Table Number, FriendlyName, @{n='GB';e={[math]::Round($_.Size/1GB,2)}} -AutoSize
        exit 1
    }
    $DiskNumber = @($cands)[0].Number
}
$disk = Get-Disk -Number $DiskNumber
if ($disk.Number -eq 0) { Say '!! 拒绝操作磁盘 0' 'Red'; exit 1 }
if ($disk.BusType -ne 'USB') { Say '!! 目标不是 USB，拒绝继续' 'Red'; exit 1 }

if (-not $DriveLetter) {
    $part = Get-Partition -DiskNumber $DiskNumber -EA SilentlyContinue |
            Where-Object { $_.DriveLetter } | Select-Object -First 1
    if ($part) { $DriveLetter = [string]$part.DriveLetter }
}
if (-not $DriveLetter) { Say '!! 这块盘没盘符，用 -DriveLetter E 指定' 'Red'; exit 1 }
$VOL = "\\.\${DriveLetter}:"

Say ''
Say ("磁盘 {0}   {1}   {2} GB   {3}" -f $disk.Number, $disk.FriendlyName,
     [math]::Round($disk.Size/1GB,2), $disk.BusType)
Say ("卷   {0}" -f $VOL)
Say ("写入 物理扇区 {0}（偏移 {1}，{2:N2} MB）" -f $TARGET_SECTOR, $OFFSET, ($OFFSET/1MB))

# ============================================================ [1] 安全检查
Head '安全检查：目标扇区是否和现有文件重叠'

# 分区起点
$partOffset = (Get-Partition -DiskNumber $DiskNumber | Select-Object -First 1).Offset
$partStart = [int]($partOffset / $SECTOR)
Say ("分区起始扇区 : {0}" -f $partStart)

# IMPORTANT: \.\E: is a VOLUME handle, so its offsets are relative to the
# partition start.  The FPGA SD controller uses absolute card LBAs.  Convert
# the desired absolute sector 131072 to the volume-relative byte offset here.
if ($TARGET_SECTOR -lt $partStart) {
    Say '!! 目标扇区位于分区之前，无法通过卷句柄写入' 'Red'
    exit 1
}
$OFFSET = [int64]($TARGET_SECTOR - $partStart) * $SECTOR
Say ("卷内偏移     : {0} 字节（物理扇区 {1} -> 分区内扇区 {2}）" -f $OFFSET, $TARGET_SECTOR, ($TARGET_SECTOR - $partStart))

# 引导扇区
$fs = $null
for ($i = 0; $i -lt 5; $i++) {
    try {
        $fs = [System.IO.File]::Open($VOL, [System.IO.FileMode]::Open,
                [System.IO.FileAccess]::Read, [System.IO.FileShare]::ReadWrite)
        break
    } catch { Start-Sleep -Seconds 3 }
}
if (-not $fs) { Say '!! 打不开卷（被占用），重跑一次' 'Red'; exit 1 }
try {
    $bs = New-Object byte[] 512
    $fs.Seek(0, 'Begin') | Out-Null
    $fs.Read($bs, 0, 512) | Out-Null
} finally { $fs.Close() }

$bps  = [BitConverter]::ToUInt16($bs, 11)
$spc  = [int]$bs[13]
$rsvd = [BitConverter]::ToUInt16($bs, 14)
$nfat = [int]$bs[16]
$fsz  = [BitConverter]::ToUInt32($bs, 36)
$dataStart = $rsvd + ($nfat * $fsz)
Say ("FAT 几何     : {0} B/扇区, {1} 扇区/簇, 保留 {2}, FAT {3}x{4}, 数据区起 {5}" -f $bps, $spc, $rsvd, $nfat, $fsz, $dataStart)

function ClusToPhys([long]$c) { return $partStart + $dataStart + ($c - 2) * $spc }

$maxEndSec = 0
$unmapped = 0
foreach ($f in (Get-ChildItem "${DriveLetter}:\" -Force -File | Sort-Object Name)) {
    if ($f.Length -eq 0) { continue }
    $r = [RawVol]::Extent($f.FullName).Split('|')
    $fl = [long]$r[0]; $ll = [long]$r[1]
    if ($fl -lt 0 -or $ll -lt 0) { $unmapped++; continue }
    $e = ClusToPhys $ll + $spc - 1
    if ($e -gt $maxEndSec) { $maxEndSec = $e }
}
Say ("现有文件最高物理扇区 : {0}" -f $maxEndSec)
Say ("音频目标扇区         : {0}" -f $TARGET_SECTOR)

if ($unmapped -gt 0) {
    Say ("!! 有 {0} 个文件映射不出来，不能确定安全。请先排查。" -f $unmapped) 'Red'
    exit 1
}
if ($TARGET_SECTOR -le $maxEndSec) {
    Say '!! 危险：目标区和现有文件重叠，写下去会破坏文件。中止。' 'Red'
    exit 1
}
Say ("✅ 安全：中间空 {0} 扇区（{1:N1} MB）" -f ($TARGET_SECTOR - $maxEndSec - 1), (($TARGET_SECTOR - $maxEndSec - 1) * $bps / 1MB)) 'Green'

if ($CheckOnly) { Say ''; Say '只检查，退出。' 'Yellow'; exit 0 }

# ============================================================ 2] 是否已存在
Head '卡上现在有没有音频'
$cmp = [RawVol]::CompareAt($VOL, $OFFSET, $bytes)
$cf = $cmp.Split('|')
$alreadyOk = $false
if ($cf[0] -eq 'OK') {
    $ndiff = [int]$cf[1]
    Say ("读回比对：不同 {0} 字节" -f $ndiff)
    if ($ndiff -eq 0) { $alreadyOk = $true; Say '✅ 卡上音频已经是正确的' 'Green' }
    else { Say ("❌ 音频缺失或已损坏（第一个不同在第 {0} 字节）" -f $cf[2]) 'Red' }
} else {
    Say ("读取失败：{0}" -f $cmp) 'Yellow'
}

if ($VerifyOnly) {
    exit $(if ($alreadyOk) { 0 } else { 1 })
}
if ($alreadyOk -and -not $Yes) {
    Say ''
    Say '已经是对的，不需要重写。要强制重写就加 -Yes' 'Yellow'
    exit 0
}

# ============================================================ [3] 备份
Head '备份目标区域'
try {
    $bk = [RawVol]::DumpAt($VOL, [int64]$OFFSET - (512 * $SECTOR), $BACKUP_SECTORS * $SECTOR, $BACKUP)
    if ($bk -like 'OK|*') { Say ("已备份 -> {0}" -f $BACKUP) 'Green' }
    else { Say ("备份跳过：{0}" -f $bk) 'Yellow' }
} catch { Say ("备份跳过：{0}" -f $_.Exception.Message) 'Yellow' }

# ============================================================ [4] 写
if (-not $isAdmin) {
    Say ''
    Say '!! 写入需要管理员权限。请以管理员身份重开 PowerShell 再跑一次。' 'Red'
    exit 1
}
Head '写入'
$res = [RawVol]::Write($VOL, $OFFSET, $bytes)
$rf = $res.Split('|')
if ($rf[0] -ne 'OK') {
    Say ("❌ 写入失败：{0}" -f $res) 'Red'
    if ($res -like 'ERRLOCK*') { Say '   -> 卷被占用。关掉资源管理器对 E: 的窗口 / 拔插卡后立刻重试' 'Yellow' }
    elseif ($res -like 'ERRopen*5*') { Say '   -> 拒绝访问，确认是管理员窗口' 'Yellow' }
    elseif ($res -like 'ERRwrite*') { Say '   -> 卡可能写保护或损坏' 'Yellow' }
    exit 1
}
Say ("✅ 写入成功，{0} 字节" -f $rf[1]) 'Green'

# ============================================================ [4b] 清掉尾部老音频
#  [2026-10-24] 为什么要多做这一步
#  ---------------------------------------------------------------------------
#  这块卡上以前写过【长版本】的音频布局：
#      alarm 764 扇区 @131072，ring 757 扇区 @131836（合计 1521 扇区）
#  现在只用 720 扇区（131072..131791）。也就是说 131792..132592 这 800 多个
#  扇区里【仍然躺着老版本的音频数据】。
#
#  正常情况下 RTL 不会读到那里（每段都有 base/sectors 夹紧），但一旦哪一版
#  固件、或者哪一次异常跑到了窗口外面，读过去就会放出"很早以前那个 MP3 版
#  的声音"，而且完全不可复现 —— 上次上板就撞到过一次。
#
#  所以写完之后，把 [TARGET+720, TARGET+2048) 这 1328 个扇区（约 680 KB）
#  全部清零。这个区间在 FAT32 看来是空闲的（现有图片最高到扇区 63720），
#  清零没有任何副作用；但万一以后又读到窗口外面，听到的就是【静音】，
#  而不是莫名其妙的老音频。
$WIPE_FROM = $TARGET_SECTOR + 720          # 当前布局的结尾
$WIPE_NUM  = 1328                          # 约 680 KB，覆盖老的 1521 扇区布局还有余量
$wipeOff   = [int64]($WIPE_FROM - $partStart) * $SECTOR
Head '清除尾部老音频残留'
try {
    $zeros = New-Object byte[] ($WIPE_NUM * $SECTOR)
    $wr = [RawVol]::Write($VOL, $wipeOff, $zeros)
    if ($wr -like 'OK|*') {
        Say ("✅ 已清零 物理扇区 {0} .. {1}（{2} 扇区）" -f $WIPE_FROM, ($WIPE_FROM + $WIPE_NUM - 1), $WIPE_NUM) 'Green'
    } else {
        Say ("清零失败（不影响音频本体）：{0}" -f $wr) 'Yellow'
    }
} catch { Say ("清零跳过：{0}" -f $_.Exception.Message) 'Yellow' }

# ============================================================ [5] 校验
Head '校验（读回逐字节比对）'
Start-Sleep -Seconds 2
$cmp2 = [RawVol]::CompareAt($VOL, $OFFSET, $bytes)
$c2 = $cmp2.Split('|')
if ($c2[0] -ne 'OK') { Say ("读回失败：{0}" -f $cmp2) 'Red'; exit 1 }
if ([int]$c2[1] -ne 0) {
    Say ("❌ 校验失败：不同 {0} 字节，第一个在第 {1} 字节" -f $c2[1], $c2[2]) 'Red'
    exit 1
}
Say ('✅ 逐字节完全一致' ) 'Green'

$tmp = Join-Path $env:TEMP 'sd_readback.bin'
$dp = [RawVol]::DumpAt($VOL, $OFFSET, $bytes.Length, $tmp)
if ($dp -like 'OK|*') {
    $shaRb = (Get-FileHash -LiteralPath $tmp -Algorithm SHA256).Hash
    Say ("源   SHA256 : {0}" -f $shaSrc) 'Gray'
    Say ("读回 SHA256 : {0}" -f $shaRb) 'Gray'
    if ($shaRb -eq $shaSrc) { Say '✅ SHA256 一致' 'Green' }
    else { Say '❌ SHA256 不一致' 'Red'; exit 1 }
}

Head '文件系统是否完好'
Start-Sleep -Seconds 2
$n = @(Get-ChildItem "${DriveLetter}:\" -Force -File -EA SilentlyContinue).Count
Say ("{0}:\ 下文件数 = {1}" -f $DriveLetter, $n)
if ($n -lt 12) { Say '!! 文件数变少了，检查一下！' 'Red'; exit 1 }
Say '✅ BMP 文件都在' 'Green'

Say ''
Say '===== 完成：音频已写入并校验通过，可以上板 =====' 'Green'
exit 0
