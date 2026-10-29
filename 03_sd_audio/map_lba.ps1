# Map a file to its physical LBA using the filesystem driver
# (FSCTL_GET_RETRIEVAL_POINTERS). This is ground truth, unlike my FAT parser.
$sig = @'
using System;
using System.Runtime.InteropServices;
using Microsoft.Win32.SafeHandles;

public static class FileLba {
    [StructLayout(LayoutKind.Sequential)]
    public struct STARTING_VCN_INPUT_BUFFER { public long StartingVcn; }

    [StructLayout(LayoutKind.Sequential)]
    public struct RETRIEVAL_POINTERS_BUFFER {
        public uint ExtentCount;
        public long StartingVcn;
        public long NextVcn;
        public long Lcn;     // only valid for the FIRST extent as laid out
    }

    [DllImport("kernel32.dll", SetLastError = true)]
    static extern SafeFileHandle CreateFile(string name, uint access, uint share,
        IntPtr sec, uint disp, uint flags, IntPtr tmpl);

    [DllImport("kernel32.dll", SetLastError = true)]
    static extern bool DeviceIoControl(SafeFileHandle h, uint code,
        ref STARTING_VCN_INPUT_BUFFER inBuf, uint inSize,
        IntPtr outBuf, uint outSize, out uint ret, IntPtr overlapped);

    const uint GENERIC_READ = 0x80000000;
    const uint FILE_SHARE_READ = 1, FILE_SHARE_WRITE = 2;
    const uint OPEN_EXISTING = 3;
    const uint FSCTL_GET_RETRIEVAL_POINTERS = 0x00090073;

    // returns "startVcn:lcn" pairs as a string, or "ERR:<code>"
    public static string GetExtents(string path) {
        var h = CreateFile(path, GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                           IntPtr.Zero, OPEN_EXISTING, 0, IntPtr.Zero);
        if (h.IsInvalid) return "ERR:open:" + Marshal.GetLastWin32Error();
        try {
            var inb = new STARTING_VCN_INPUT_BUFFER { StartingVcn = 0 };
            uint outSize = 4096;
            IntPtr ob = Marshal.AllocHGlobal((int)outSize);
            try {
                uint ret;
                bool ok = DeviceIoControl(h, FSCTL_GET_RETRIEVAL_POINTERS,
                    ref inb, (uint)Marshal.SizeOf(inb), ob, outSize, out ret, IntPtr.Zero);
                if (!ok) return "ERR:ioctl:" + Marshal.GetLastWin32Error();
                uint count = (uint)Marshal.ReadInt32(ob, 0);
                long startVcn = Marshal.ReadInt64(ob, 8);
                var sb = new System.Text.StringBuilder();
                sb.Append("count=").Append(count).Append(" startVcn=").Append(startVcn);
                for (int i = 0; i < count && i < 8; i++) {
                    int off = 16 + i * 16;
                    long nextVcn = Marshal.ReadInt64(ob, off);
                    long lcn     = Marshal.ReadInt64(ob, off + 8);
                    sb.Append("  [vcn ").Append(nextVcn).Append(" -> lcn ").Append(lcn).Append("]");
                }
                return sb.ToString();
            } finally { Marshal.FreeHGlobal(ob); }
        } finally { h.Close(); }
    }
}
'@
Add-Type -TypeDefinition $sig -Language CSharp

foreach ($f in (Get-ChildItem 'E:\' -Force -File | Sort-Object Name)) {
    $r = [FileLba]::GetExtents($f.FullName)
    Write-Host ("{0,-24} {1}" -f $f.Name, $r)
}
