# foc_console.ps1 - FOC_G431 CLI serial helper (auto disables telemetry, strips binary)
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\foc_console.ps1 -Cmd status
#   powershell -ExecutionPolicy Bypass -File tools\foc_console.ps1 -Cmd mode,vel,enable,"target 30",status,disable
param(
    [string]$Port = 'COM44',
    [int]$Baud = 6500000,
    [string[]]$Cmd = @('status'),
    [int]$Gap = 350,           # ms to wait/collect after each command
    [switch]$KeepTelemetry     # by default send 'log 0' first to silence binary telemetry
)

# allow -Cmd a,b,c as well as proper PowerShell arrays
$Cmd = @($Cmd | ForEach-Object { $_ -split ',' } | ForEach-Object { $_.Trim() } | Where-Object { $_ })

$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, 'None', 8, 'one')
$sp.ReadTimeout = 600

function Invoke-Ask($s, $cmd, $wait) {
    $s.Write($cmd + "`n")
    Start-Sleep -Milliseconds $wait
    $raw = ''
    for ($i = 0; $i -lt 24; $i++) {
        if ($s.BytesToRead -gt 0) { $raw += $s.ReadExisting() } else { break }
        Start-Sleep -Milliseconds 25
    }
    $clean = -join ($raw.ToCharArray() | ForEach-Object {
        $x = [int]$_
        if (($x -ge 32 -and $x -le 126) -or $x -eq 10 -or $x -eq 13) { $_ } else { ' ' }
    })
    return ($clean -replace ' {2,}', ' ').Trim()
}

try {
    $sp.Open()
    Start-Sleep -Milliseconds 120
    $sp.DiscardInBuffer()
    if (-not $KeepTelemetry) { [void](Invoke-Ask $sp 'log 0' 200) }
    foreach ($c in $Cmd) {
        Write-Host ">>> $c" -ForegroundColor Cyan
        Write-Host (Invoke-Ask $sp $c $Gap)
        Write-Host '-----'
    }
    $sp.Close()
} catch {
    Write-Host "SERIAL ERROR: $($_.Exception.Message)" -ForegroundColor Red
    try { $sp.Write("target 0`n"); Start-Sleep -Milliseconds 100; $sp.Write("disable`n") } catch {}
    if ($sp.IsOpen) { $sp.Close() }
}
