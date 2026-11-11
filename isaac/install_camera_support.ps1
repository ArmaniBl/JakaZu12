param(
    [string]$ExtensionDirectory = "$env:USERPROFILE\Documents\Kit\shared\exts\IsaacMoveitExtension\Isaac_MoveIt_Extension_python"
)
$ErrorActionPreference = 'Stop'
$entry = Join-Path $ExtensionDirectory 'extension.py'
if (!(Test-Path -LiteralPath $entry)) { throw "Extension not found: $entry" }
$source = Get-Content -LiteralPath $entry -Raw
$startup = '        self._timeline = omni.timeline.get_timeline_interface()'
$shutdown = '    def on_shutdown(self):'
if (!$source.Contains($startup) -or !$source.Contains($shutdown)) {
    throw 'Extension structure differs; no files changed.'
}
$backup = "$entry.bak_camera_$(Get-Date -Format yyyyMMdd_HHmmss)"
Copy-Item -LiteralPath $entry -Destination $backup
if (!$source.Contains('from .welding_camera_repair import start')) {
    $source = $source.Replace($startup, "$startup`n        from .welding_camera_repair import start`n        self._camera_repair_task = start()")
}
if (!$source.Contains('self._camera_repair_task.cancel()')) {
    $source = $source.Replace($shutdown, "$shutdown`n        if getattr(self, '_camera_repair_task', None):`n            self._camera_repair_task.cancel()")
}
# Install support before enabling the startup hook.
Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'camera_repair.py') -Destination (Join-Path $ExtensionDirectory 'welding_camera_repair.py')
[IO.File]::WriteAllText($entry, $source, [Text.UTF8Encoding]::new($false))
Write-Output "Camera support installed. Extension backup: $backup"
