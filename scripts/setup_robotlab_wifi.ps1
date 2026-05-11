<#
.SYNOPSIS
    Sets up a Windows PC as a WiFi access point ("robotlab-wifi") for the Jetson Orin Nano (robotlab).

.DESCRIPTION
    Creates a WiFi hotspot using the Windows hosted network feature, configures a
    static IP (10.42.0.1/24) matching robotlab's static IP (10.42.0.2/24), and enables
    NAT so robotlab can reach the internet through this PC.

    Robotlab expects:
        - SSID:     robotlab-wifi
        - Password: labrobot123
        - Gateway:  10.42.0.1
        - Static:   10.42.0.2/24

.NOTES
    Requires Windows 10 or 11 with a WiFi adapter that supports hosted network / soft AP.
    Must be run as Administrator.

    If the hosted network feature is not supported by your WiFi driver, use Windows
    Settings → Network & Internet → Mobile Hotspot instead, and set the IP manually:
        netsh interface ip set address "Local Area Connection* xx" static 10.42.0.1 255.255.255.0

    Author: Hermes Agent (asger@catla)
#>

#Requires -RunAsAdministrator

$SSID = "robotlab-wifi"
$PASSWORD = "labrobot123"
$HOST_IP = "10.42.0.1"
$PREFIX = 24

Write-Host "=== robotlab-wifi Hotspot Setup for Windows ===" -ForegroundColor Cyan
Write-Host ""

# ───── Step 1: Check if hosted network is supported ─────
Write-Host "[1/5] Checking hosted network support..." -ForegroundColor Yellow
$hostedCapability = netsh wlan show drivers | Select-String "Hosted network supported"
if ($hostedCapability -match "Yes") {
    Write-Host "  Hosted network is supported." -ForegroundColor Green
} elseif ($hostedCapability -match "No") {
    Write-Host "  WARNING: Hosted network is NOT supported by your WiFi driver." -ForegroundColor Red
    Write-Host "  Alternative: Use Windows Settings → Mobile Hotspot, or use a USB WiFi" -ForegroundColor Yellow
    Write-Host "  adapter that supports hosted network mode (e.g. Linksys AE3000)." -ForegroundColor Yellow
    exit 1
} else {
    Write-Host "  Could not determine hosted network support. Proceeding anyway..." -ForegroundColor Yellow
}

# ───── Step 2: Configure the hosted network ─────
Write-Host "[2/5] Configuring hosted network (SSID: $SSID)..." -ForegroundColor Yellow
$result = netsh wlan set hostednetwork mode=allow ssid=$SSID key=$PASSWORD 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "  Failed to configure hosted network: $result" -ForegroundColor Red
    exit 1
}
Write-Host "  Hosted network configured." -ForegroundColor Green

# ───── Step 3: Start the hosted network ─────
Write-Host "[3/5] Starting hosted network..." -ForegroundColor Yellow
$result = netsh wlan start hostednetwork 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "  Failed to start hosted network: $result" -ForegroundColor Red
    Write-Host "  Try: unplug and replug your WiFi adapter, then run again." -ForegroundColor Yellow
    exit 1
}
Write-Host "  Hosted network started." -ForegroundColor Green

# ───── Step 4: Find the virtual adapter and set static IP ─────
Write-Host "[4/5] Setting static IP on virtual adapter ($HOST_IP/$PREFIX)..." -ForegroundColor Yellow

# The hosted network creates a virtual adapter — find it
$virtualAdapter = Get-NetAdapter | Where-Object {
    $_.InterfaceDescription -match "Hosted Network Virtual|Virtual WiFi|Microsoft Wi-Fi Direct Virtual"
} | Select-Object -First 1

if (-not $virtualAdapter) {
    # Fallback: look for an adapter that recently appeared and has no IP or a 192.168.137.x IP
    Write-Host "  Could not find virtual adapter by description, scanning for it..." -ForegroundColor Yellow
    $virtualAdapter = Get-NetAdapter | Where-Object {
        $_.Status -eq "Up" -and $_.InterfaceDescription -notmatch "Bluetooth|Loopback"
    } | Sort-Object Name | Select-Object -First 1
}

if (-not $virtualAdapter) {
    Write-Host "  Could not find virtual adapter. Run this after step 3:" -ForegroundColor Yellow
    Write-Host "    netsh interface ip set address name=""<adapter_name>"" static $HOST_IP 255.255.255.0" -ForegroundColor Yellow
    Write-Host "  Then run: Get-NetAdapter | Format-Table Name, Status, InterfaceDescription" -ForegroundColor Yellow
} else {
    $ifaceName = $virtualAdapter.Name
    Write-Host "  Found virtual adapter: '$ifaceName'" -ForegroundColor Green

    # Set static IP (remove any DHCP config first)
    netsh interface ip set address name="$ifaceName" static $HOST_IP 255.255.255.0 2>&1 | Out-Null
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  IP set to $HOST_IP/24 on '$ifaceName'." -ForegroundColor Green
    } else {
        Write-Host "  Could not set IP. Try manually:" -ForegroundColor Yellow
        Write-Host "    netsh interface ip set address name=""$ifaceName"" static $HOST_IP 255.255.255.0" -ForegroundColor Yellow
    }
}

# ───── Step 5: Enable Internet Connection Sharing (NAT) ─────
Write-Host "[5/5] Setting up internet sharing..." -ForegroundColor Yellow

# Find the internet-connected adapter (the one with a default gateway)
$internetAdapter = Get-NetAdapter | Where-Object {
    $_.Status -eq "Up" -and
    (Get-NetIPConfiguration -InterfaceAlias $_.Name -ErrorAction SilentlyContinue).Ipv4DefaultGateway
} | Select-Object -First 1

if (-not $internetAdapter) {
    Write-Host "  Could not find an internet-connected adapter. Skipping NAT setup." -ForegroundColor Yellow
} else {
    Write-Host "  Internet adapter: '$($internetAdapter.Name)'" -ForegroundColor Green

    # Remove any existing NAT with the same name
    Get-NetNat -Name "robotlab-nat" -ErrorAction SilentlyContinue | Remove-NetNat -Confirm:$false
    Get-NetNat -Name "robotlab-wifi-nat" -ErrorAction SilentlyContinue | Remove-NetNat -Confirm:$false

    # Create NAT for the hotspot subnet
    New-NetNat -Name "robotlab-wifi-nat" -InternalIPInterfaceAddressPrefix "10.42.0.0/24" -Verbose
    if ($LASTEXITCODE -eq 0) {
        Write-Host "  NAT enabled — robotlab can now reach the internet through this PC." -ForegroundColor Green
    } else {
        Write-Host "  Could not create NAT. Enable ICS manually:" -ForegroundColor Yellow
        Write-Host "  1. Open Control Panel → Network and Sharing Center → Change adapter settings" -ForegroundColor Yellow
        Write-Host "  2. Right-click your internet adapter → Properties → Sharing tab" -ForegroundColor Yellow
        Write-Host "  3. Check ""Allow other network users to connect"" and select '$ifaceName'" -ForegroundColor Yellow
    }
}

# ───── Summary ─────
Write-Host ""
Write-Host "=== Setup Summary ===" -ForegroundColor Cyan
netsh wlan show hostednetwork

Write-Host ""
Write-Host "Internet sharing:" -ForegroundColor Cyan
Get-NetNat -Name "robotlab-wifi-nat" -ErrorAction SilentlyContinue | Format-Table Name, InternalIPInterfaceAddressPrefix

Write-Host ""
Write-Host "SSH into robotlab from this machine:" -ForegroundColor Green
Write-Host "  ssh robotlab@10.42.0.2" -ForegroundColor White
Write-Host ""

Write-Host "To stop the hotspot:" -ForegroundColor Yellow
Write-Host "  netsh wlan stop hostednetwork" -ForegroundColor White
Write-Host ""
Write-Host "To remove it completely:" -ForegroundColor Yellow
Write-Host "  netsh wlan set hostednetwork mode=disallow" -ForegroundColor White
