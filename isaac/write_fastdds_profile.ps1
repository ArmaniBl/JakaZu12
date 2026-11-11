param(
    [Parameter(Mandatory = $true)]
    [string]$WindowsIp,
    [Parameter(Mandatory = $true)]
    [string]$WslIp
)

$ports = 7410..7450

function New-LocatorList($Address) {
    ($ports | ForEach-Object {
@"
                    <locator>
                        <udpv4>
                            <address>$Address</address>
                            <port>$_</port>
                        </udpv4>
                    </locator>
"@
    }) -join "`r`n"
}

$peerLocators = New-LocatorList -Address $WslIp

$xml = @"
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>welding_udp</transport_id>
            <type>UDPv4</type>
            <maxMessageSize>1400</maxMessageSize>
            <sendBufferSize>16777216</sendBufferSize>
            <receiveBufferSize>16777216</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="isaac_sim_participant" is_default_profile="true">
        <rtps>
            <userTransports><transport_id>welding_udp</transport_id></userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
            <builtin>
                <initialPeersList>
$peerLocators
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
"@

$out = Join-Path $env:USERPROFILE ".ros\fastdds.xml"
New-Item -ItemType Directory -Force -Path (Split-Path -Parent $out) | Out-Null
Set-Content -LiteralPath $out -Value $xml -Encoding ASCII
