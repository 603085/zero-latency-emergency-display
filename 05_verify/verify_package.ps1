# ============================================================================
#  verify_package.ps1  --  check the ABC handoff package is intact
#
#  1) 01_project\bit\TaskC_v7f.bit        sha256
#  2) 03_sd_audio\audio_sd.bin            sha256
#  3) 01_project\src\*  against 01_project\SRC_MANIFEST.sha256
#  4) 02_sd_images\  contains exactly 12 .bmp files
#
#  Uses $PSScriptRoot so it works wherever the folder is unpacked.
#  Every literal here is ASCII-only on purpose: a .ps1 without a BOM is read
#  as GBK on a Chinese Windows, which would corrupt any non-ASCII path.
# ============================================================================
$ErrorActionPreference = 'Stop'
$root = Split-Path $PSScriptRoot -Parent
$script:fail = 0

function Check($label, $path, $want) {
    if (-not (Test-Path $path)) { Write-Host "$label  MISSING  $path"; $script:fail = 1; return }
    $got = (Get-FileHash $path -Algorithm SHA256).Hash
    if ($got -eq $want) { Write-Host "$label  OK   $got" }
    else                { Write-Host "$label  BAD  $got"; Write-Host "           want $want"; $script:fail = 1 }
}

Check 'BIT      ' (Join-Path $root '01_project\bit\TaskC_v7f.bit') `
      'EA78381121BCDB397F7A314E67D911BA47860979370F1EB79603A552A95372F6'

Check 'SD AUDIO ' (Join-Path $root '03_sd_audio\audio_sd.bin') `
      '05A5995EC59CE13ED049E32B1233B7C50D23912E9814FB011326C6EA113C6BE4'

$bad = 0
$n   = 0
$man = Join-Path $root '01_project\SRC_MANIFEST.sha256'
foreach ($line in Get-Content $man) {
    if ($line -match '^([0-9a-f]{64})\s+src/(.+)$') {
        $h = $Matches[1]; $name = $Matches[2]; $n++
        $p = Join-Path $root ("01_project\src\" + $name)
        if (-not (Test-Path $p)) { Write-Host "SRC  MISSING $name"; $bad++; continue }
        $g = (Get-FileHash $p -Algorithm SHA256).Hash.ToLower()
        if ($g -ne $h) { Write-Host "SRC  BAD     $name"; $bad++ }
    }
}
if ($bad -eq 0) { Write-Host "SRC        OK   $n files match the manifest" }
else            { Write-Host "SRC        BAD  $bad file(s)"; $script:fail = 1 }

$img = Join-Path $root '02_sd_images'
$bmp = 0
if (Test-Path $img) { $bmp = (Get-ChildItem $img -Filter *.bmp -File).Count }
if ($bmp -eq 12) { Write-Host "BMP        OK   12 pictures" }
else             { Write-Host "BMP        BAD  found $bmp (expect 12)"; $script:fail = 1 }

Write-Host ""
if ($script:fail -ne 0) { Write-Host "PACKAGE HAS PROBLEMS."; exit 1 }
else                    { Write-Host "PACKAGE INTACT." }
