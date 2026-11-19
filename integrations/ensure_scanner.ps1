$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
$base = 'http://127.0.0.1:8158'
function ScannerReady {
    try { $health = Invoke-RestMethod "$base/api/health" -TimeoutSec 1; return $null -ne $health }
    catch { return $false }
}
if (ScannerReady) { [Console]::Write('{"ready":true,"started":false}'); exit 0 }
$station = Join-Path $env:LOCALAPPDATA 'WeldingStation'
$scannerRoot = Join-Path $station 'scanner'
$python = Join-Path $scannerRoot '.venv/Scripts/python.exe'
$server = Join-Path $station 'windows_server.py'
if (!(Test-Path -LiteralPath $python) -or !(Test-Path -LiteralPath $server)) {
    throw 'Prepared Windows scanner installation is missing. Start Welding - Scanner manually.'
}
$mutex = New-Object System.Threading.Mutex($false, 'Local\WeldingStationScannerStart')
$locked = $false
try {
    $locked = $mutex.WaitOne(5000)
    if (!$locked) { throw 'Another scanner startup is still in progress' }
    if (ScannerReady) { [Console]::Write('{"ready":true,"started":false}'); exit 0 }
    $logRoot = Join-Path $station 'logs'
    New-Item -ItemType Directory -Path $logRoot -Force | Out-Null
    $arguments = @('-X','utf8',('"'+$server+'"'),'--scanner-root',('"'+$scannerRoot+'"'))
    $process = Start-Process -FilePath $python -ArgumentList $arguments -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $logRoot 'scanner-service.log') `
        -RedirectStandardError (Join-Path $logRoot 'scanner-service.err.log')
    $deadline = [DateTime]::UtcNow.AddSeconds(30)
    while (!(ScannerReady)) {
        if ($process.HasExited) { throw 'Scanner service exited. Check WeldingStation/logs/scanner-service.err.log' }
        if ([DateTime]::UtcNow -gt $deadline) { throw 'Windows scanner startup timed out' }
        Start-Sleep -Milliseconds 250
    }
    $loaded = $false
    $state = Invoke-RestMethod "$base/api/state" -TimeoutSec 5
    $project = Join-Path $scannerRoot 'runtime/scanner/project.json'
    if ($state.parts.Count -eq 0 -and (Test-Path -LiteralPath $project)) {
        $restored = Invoke-RestMethod "$base/api/project/load" -Method Post -ContentType 'application/json' -Body '{}' -TimeoutSec 15
        $loaded = $restored.parts.Count -gt 0
    }
    [Console]::Write((@{ready=$true;started=$true;project_loaded=$loaded} | ConvertTo-Json -Compress))
} finally {
    if ($locked) { $mutex.ReleaseMutex() }
    $mutex.Dispose()
}
