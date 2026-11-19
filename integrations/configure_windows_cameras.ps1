param(
    [Parameter(Mandatory=$true)][string]$Cti,
    [Parameter(Mandatory=$true)][string]$Camera0,
    [Parameter(Mandatory=$true)][string]$Camera1,
    [string]$DllDirectories = '',
    [string]$TablePort = ''
)
$ErrorActionPreference = 'Stop'
if (-not (Test-Path -LiteralPath $Cti -PathType Leaf)) { throw 'CTI file does not exist' }
if ($Camera0 -eq $Camera1) { throw 'Two distinct camera serial numbers are required' }
foreach ($value in @($Cti,$Camera0,$Camera1,$DllDirectories,$TablePort)) {
    if ($value -match '["\r\n%&|<>^]') { throw 'Unsupported characters in camera configuration' }
}
if ($TablePort -and $TablePort -notmatch '^COM[0-9]+$') { throw 'Expected COM port, for example COM3' }
$root = Join-Path $env:LOCALAPPDATA 'WeldingStation'
$config = Join-Path $root 'scanner\config.cmd'
if (Test-Path -LiteralPath $config) {
    Copy-Item -LiteralPath $config -Destination ($config+'.backup-'+(Get-Date -Format yyyyMMdd_HHmmss))
}
$text = @"
@set "SPECTRUM_GENTL_CTI=$Cti"
@set "SPECTRUM_SDK_DLL_DIRS=$DllDirectories"
@set "SPECTRUM_CAMERA_0_SERIAL=$Camera0"
@set "SPECTRUM_CAMERA_1_SERIAL=$Camera1"
@set "SPECTRUM_TABLE_PORT=$TablePort"
"@
[IO.File]::WriteAllText($config,$text,(New-Object System.Text.UTF8Encoding($false)))
Write-Output 'Saved config.cmd. Restart the scanner, then run Welding - Diagnose.'
