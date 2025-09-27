# Sends a restart countdown message via InSim before restarting the service.
param(
    [int]$CountdownSeconds = 30,
    [string]$ConfigPath = 'C:\\AHPP_INSIM\\config.ini',
    [string]$DllPath = 'C:\\AHPP_INSIM\\bin\\Release\\InSimDotNet.dll'
)

$ErrorActionPreference = 'Stop'

if (-not (Test-Path $ConfigPath)) {
    throw "Config file not found: $ConfigPath"
}
if (-not (Test-Path $DllPath)) {
    throw "InSimDotNet.dll not found: $DllPath"
}

Add-Type -Path $DllPath

# Parse INI file to extract [Insim] section
$ini = @{}
$section = ''
foreach ($line in Get-Content $ConfigPath) {
    $trim = $line.Trim()
    if ($trim -eq '' -or $trim.StartsWith(';')) { continue }
    if ($trim.StartsWith('[') -and $trim.EndsWith(']')) {
        $section = $trim.Trim('[', ']')
        if (-not $ini.ContainsKey($section)) { $ini[$section] = @{} }
        continue
    }
    if ($section -and $trim -match '^(.*?)\s*=\s*(.*)$') {
        $key = $matches[1].Trim()
        $value = $matches[2].Trim().Trim('"')
        $ini[$section][$key] = $value
    }
}

$insimSection = $ini['Insim']

$settings = [InSimDotNet.InSimSettings]@{
    Host  = $insimSection['addr']
    Port  = [int]$insimSection['port']
    Admin = $insimSection['pass']
}

$insim = [InSimDotNet.InSim]::new()
$insim.Initialize($settings)

for ($i = $CountdownSeconds; $i -gt 0; $i--) {
    if ($i -eq $CountdownSeconds -or $i % 5 -eq 0) {
        $insim.Send([InSimDotNet.Packets.IS_MST]@{ Msg = "Insim restarting in $i seconds..." })
    }
    Start-Sleep -Seconds 1
}
$insim.Send([InSimDotNet.Packets.IS_MST]@{ Msg = 'Check changelog in [MENU] -> Settings' })
$insim.Send([InSimDotNet.Packets.IS_MST]@{ Msg = 'Insim restarting now! Bbbwwwuuueee...' })
$insim.Disconnect()
$insim.Dispose()
