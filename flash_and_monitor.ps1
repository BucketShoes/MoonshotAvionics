# flash_and_monitor.ps1
# Build + upload one PlatformIO environment, optionally upload LittleFS, then monitor.
#
#   .\flash_and_monitor.ps1 -Env moonbase_lora32v43 -Label "BASE STATION" -Fs
#   .\flash_and_monitor.ps1 -Env moonshot_trackerv1 -Label "ROCKET"
#
# The COM port is read out of platformio.ini for the given environment, so
# there is exactly one place to change it. -Port overrides if you need to.

param(
    [Parameter(Mandatory = $true)][string]$Env,
    [string]$Label = $Env,
    [string]$Port,
    [switch]$Fs          # also compress web assets and upload the LittleFS image
)

$ErrorActionPreference = "Stop"
$PIO = "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe"

function Fail($msg, $code) {
    Write-Host ""
    Write-Host "$msg"
    Read-Host "Press Enter to close"
    exit $code
}

# ---- Resolve the upload port from platformio.ini ----------------------------
if (-not $Port) {
    $ini      = Get-Content "$PSScriptRoot\platformio.ini"
    $inTarget = $false
    foreach ($line in $ini) {
        if ($line -match '^\s*\[env:(.+?)\]\s*$') { $inTarget = ($Matches[1] -eq $Env); continue }
        if ($inTarget -and $line -match '^\s*upload_port\s*=\s*(\S+)') { $Port = $Matches[1]; break }
    }
}
if (-not $Port) { Fail "Could not find upload_port for env '$Env' in platformio.ini." 1 }

Write-Host "=== $Label : env=$Env port=$Port ==="

# ---- Release the port from any monitor still holding it ---------------------
Get-CimInstance Win32_Process -Filter "Name='python.exe' OR Name='python3.exe'" |
    Where-Object { $_.CommandLine -match [regex]::Escape($Port) } |
    ForEach-Object {
        Write-Host "Killing existing monitor PID $($_.ProcessId)"
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 500

# ---- Web assets (LittleFS builds only) --------------------------------------
if ($Fs) {
    Write-Host ""
    Write-Host "=== $Label : compressing web assets ==="
    & powershell -NoProfile -ExecutionPolicy Bypass -File "$PSScriptRoot\base_station\compress_web.ps1"
    if ($LASTEXITCODE -ne 0) { Fail "Compress FAILED (exit $LASTEXITCODE)" $LASTEXITCODE }
}

# ---- Firmware ---------------------------------------------------------------
Write-Host ""
Write-Host "=== $Label : uploading firmware ==="
& $PIO run -e $Env -t upload
if ($LASTEXITCODE -ne 0) { Fail "Firmware upload FAILED (exit $LASTEXITCODE)" $LASTEXITCODE }

function Wait-Port($port, $seconds) {
    Write-Host ""
    Write-Host "=== Waiting for $port ==="
    $deadline = (Get-Date).AddSeconds($seconds)
    while ((Get-Date) -lt $deadline) {
        if ([System.IO.Ports.SerialPort]::GetPortNames() -contains $port) {
            Write-Host "$port is up."
            return $true
        }
        Start-Sleep -Milliseconds 300
    }
    Write-Host "WARNING: $port did not reappear, continuing anyway."
    return $false
}

Wait-Port $Port 15 | Out-Null

# ---- LittleFS ---------------------------------------------------------------
if ($Fs) {
    Write-Host ""
    Write-Host "=== $Label : uploading LittleFS ==="
    & $PIO run -e $Env -t uploadfs
    if ($LASTEXITCODE -ne 0) { Write-Host "LittleFS upload FAILED (exit $LASTEXITCODE), starting monitor anyway." }
    Wait-Port $Port 15 | Out-Null
}

# ---- Monitor ----------------------------------------------------------------
Clear-Host
Write-Host "=== $Label : monitor on $Port (Ctrl+C to stop) ==="
& $PIO device monitor -e $Env
