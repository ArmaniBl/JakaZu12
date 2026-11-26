param([string]$Script, [string]$Capture, [string]$Output)
$ErrorActionPreference = 'Stop'
$python = Join-Path $env:LOCALAPPDATA 'WeldingStation/scanner/.venv/Scripts/python.exe'
& $python -X utf8 $Script $Capture $Output
exit $LASTEXITCODE
