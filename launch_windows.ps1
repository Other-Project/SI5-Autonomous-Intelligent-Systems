# ----------------------------------------------
# OAK-D Pro Auto-Attach Script for WSL (Windows)
# Device boot PID: 03e7:2485
# Device final PID: 03e7:f63b
# ----------------------------------------------

$BootUSB = "03e7:2485"
$FinalUSB = "03e7:f63b"

Write-Host "`n[OAK-D] Auto-attach script running..." -ForegroundColor Cyan

function Get-UsbipdDevice {
    usbipd list | Select-String -Pattern $BootUSB, $FinalUSB
}

$attached = $false

while ($true) {
    $device = Get-UsbipdDevice

    if ($device) {
        $columns = ($device -split "\s+")
        $busid   = $columns[0]
        $usbId   = $columns[1]  # variable renommÃ©e

        if ($usbId -eq $BootUSB) {

            Write-Host "[OAK-D] Bootloader detected ($BootUSB) at busid $busid. Waiting for transition..." -ForegroundColor Yellow

            if ($attached -eq $false) {
                usbipd bind --busid $busid
                $attach = usbipd attach --wsl --busid $busid 2>&1

                if ($LASTEXITCODE -eq 0) {
                    Write-Host "[OAK-D] Successfully attached to WSL!" -ForegroundColor Green
                    $attached = $true
                }
                else {
                    Write-Host "[OAK-D] Failed to attach: $attach" -ForegroundColor Red
                }
            }
        }
        elseif ($usbId -eq $FinalUSB) {

            Write-Host "[OAK-D] Device ready ($FinalUSB) at busid $busid." -ForegroundColor Green
            $attached = $false

            # Try attaching to WSL
            usbipd bind --busid $busid
            $attach = usbipd attach --wsl --busid $busid 2>&1

            if ($LASTEXITCODE -eq 0) {
                Write-Host "[OAK-D] Successfully attached to WSL!" -ForegroundColor Green
            }
            else {
                Write-Host "[OAK-D] Failed to attach: $attach" -ForegroundColor Red
            }

            # Wait until device disappears before looping again
            while (Get-UsbipdDevice) {
                Start-Sleep -Milliseconds 200
            }

            Write-Host "[OAK-D] Device removed. Waiting for next connection..." -ForegroundColor DarkCyan
        }
    }

    Start-Sleep -Milliseconds 200
}