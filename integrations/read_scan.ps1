param(
    [ValidateSet('export','state','capture','new-task')][string]$Action = 'export',
    [string]$TaskName,
    [string]$PartId
)
$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
try {
    $route = @{export='moveit-export'; state='state'; capture='capture'; 'new-task'='tasks'}[$Action]
    $request = @{UseBasicParsing=$true; Uri="http://127.0.0.1:8158/api/spectrum/$route"; TimeoutSec=15}
    if ($Action -eq 'capture') {
        $request.Method = 'POST'
        $request.ContentType = 'application/json'
        $request.Body = '{"require_setup":true,"turntable":false,"positions":1}'
    }
    if ($Action -eq 'new-task') {
        $request.Method = 'POST'
        $request.ContentType = 'application/json'
        $request.Body = @{name=$TaskName; part_id=$PartId} | ConvertTo-Json -Compress
    }
    $response = Invoke-WebRequest @request
    [Console]::Write($response.Content)
} catch {
    $detail = $_.ErrorDetails.Message
    if (-not $detail) { $detail = $_.Exception.Message }
    [Console]::Error.WriteLine($detail)
    exit 1
}
