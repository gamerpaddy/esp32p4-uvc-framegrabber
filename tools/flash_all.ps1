# Build everything and flash the P4, including the ESP32-C6 coprocessor images.
#
#   .\tools\flash_all.ps1 -Port COM7
#
# Order matters: the C6 firmware must exist before it can be packed, and the
# packed container must be on flash before the P4's `c6flash` command can use
# it. The C6 itself is programmed afterwards, from the P4 console.

param(
    [Parameter(Mandatory = $true)][string]$Port,
    [switch]$SkipC6Firmware
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

# c6_fw partition offset — keep in sync with partitions.csv
$C6_FW_OFFSET = "0x310000"

Write-Host "`n=== 1/4  ESP32-C6 coprocessor firmware ===" -ForegroundColor Cyan
if ($SkipC6Firmware) {
    Write-Host "skipped (-SkipC6Firmware)"
} else {
    Set-Location "$root\c6_firmware"
    idf.py set-target esp32c6
    if (-not $?) { throw "C6 set-target failed" }
    idf.py build
    if (-not $?) { throw "C6 build failed" }
    Set-Location $root
}

Write-Host "`n=== 2/4  Pack C6 images ===" -ForegroundColor Cyan
python tools\pack_c6_fw.py
if (-not $?) { throw "packing failed" }

Write-Host "`n=== 3/4  Build + flash the P4 app ===" -ForegroundColor Cyan
idf.py -p $Port flash
if (-not $?) { throw "P4 flash failed" }

Write-Host "`n=== 4/4  Write the C6 container to the c6_fw partition ===" -ForegroundColor Cyan
python -m esptool --chip esp32p4 -p $Port write-flash $C6_FW_OFFSET build\c6_fw.bin
if (-not $?) { throw "c6_fw flash failed" }

Write-Host @"

Done. Now program the C6 itself, from the P4 console:

    idf.py -p $Port monitor

    p4> c6boot -d      # confirm the C6 enters ROM download mode
    p4> c6flash        # write the coprocessor firmware (~15 s)

Then reset the P4 so ESP-Hosted brings the SDIO link up against the new
firmware, and check it worked:

    p4> status
    p4> scan
    p4> join <ssid> <password>
    p4> sdiospeed
    p4> wifispeed <pc-ip>     # with tools/../tcp_sink.py running on the PC
"@ -ForegroundColor Green
