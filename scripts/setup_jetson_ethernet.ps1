# ─────────────────────────────────────────────────────────────
# setup_jetson_ethernet.ps1 — Windows PowerShell (Run as Admin)
# ─────────────────────────────────────────────────────────────
# Configures the Ethernet adapter connected to the Jetson Orin Nano
# with a static IP of 10.42.0.1/24 so that:
#   - Windows can reach the Jetson at 10.42.0.2
#   - WSL2 (mirrored mode) inherits the interface
#   - Docker/Podman containers with --network host can reach the Jetson
#
# Usage (from an elevated PowerShell):
#   powershell -ExecutionPolicy Bypass -File scripts\setup_jetson_ethernet.ps1
#
# Or with a specific adapter:
#   powershell -ExecutionPolicy Bypass -File scripts\setup_jetson_ethernet.ps1 -InterfaceAlias "Ethernet 2"
# ─────────────────────────────────────────────────────────────
param(
    [string]$InterfaceAlias = ""
)

$TargetIP = "10.42.0.1"
$PrefixLength = 24
$JetsonIP = "10.42.0.2"

Write-Host "=== Jetson Ethernet Setup ===" -ForegroundColor Cyan

# ─── Find the Ethernet adapter ───
if ($InterfaceAlias -eq "") {
    Write-Host ""
    Write-Host "Available network adapters:" -ForegroundColor Yellow
    Get-NetAdapter | Format-Table Name, InterfaceDescription, Status, LinkSpeed -AutoSize
    Write-Host ""
    $InterfaceAlias = Read-Host "Enter the name of the Ethernet adapter connected to the Jetson"
}

# Validate the adapter exists
$adapter = Get-NetAdapter -Name $InterfaceAlias -ErrorAction SilentlyContinue
if (-not $adapter) {
    Write-Host "ERROR: Adapter '$InterfaceAlias' not found." -ForegroundColor Red
    exit 1
}

if ($adapter.Status -ne "Up") {
    Write-Host "WARNING: Adapter '$InterfaceAlias' is not Up (status: $($adapter.Status))." -ForegroundColor Yellow
    Write-Host "Make sure the Ethernet cable is connected to the Jetson and both are powered on." -ForegroundColor Yellow
}

Write-Host "[OK] Using adapter: $InterfaceAlias ($($adapter.InterfaceDescription))" -ForegroundColor Green

# ─── Remove any existing IP on this adapter in the 10.42.0.0/24 range ───
$existingIPs = Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -like "10.42.0.*" }

foreach ($ip in $existingIPs) {
    Write-Host "Removing existing IP $($ip.IPAddress) from $InterfaceAlias..." -ForegroundColor Yellow
    Remove-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $ip.IPAddress -Confirm:$false -ErrorAction SilentlyContinue
}

# ─── Set static IP ───
Write-Host "Setting $TargetIP/$PrefixLength on $InterfaceAlias..." -ForegroundColor Cyan
New-NetIPAddress -InterfaceAlias $InterfaceAlias -IPAddress $TargetIP -PrefixLength $PrefixLength -ErrorAction Stop | Out-Null
Write-Host "[OK] Static IP set: $TargetIP/$PrefixLength" -ForegroundColor Green

# ─── Remove any default gateway on this interface (point-to-point, no internet) ───
$routes = Get-NetRoute -InterfaceAlias $InterfaceAlias -DestinationPrefix "0.0.0.0/0" -ErrorAction SilentlyContinue
foreach ($route in $routes) {
    Remove-NetRoute -InterfaceAlias $InterfaceAlias -DestinationPrefix "0.0.0.0/0" -Confirm:$false -ErrorAction SilentlyContinue
    Write-Host "[OK] Removed default gateway from $InterfaceAlias (not needed for Jetson link)" -ForegroundColor Green
}

# ─── Verify ───
Write-Host ""
Write-Host "=== Verification ===" -ForegroundColor Cyan
$ip = Get-NetIPAddress -InterfaceAlias $InterfaceAlias -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -eq $TargetIP }
if ($ip) {
    Write-Host "[OK] IP configured: $($ip.IPAddress)/$($ip.PrefixLength)" -ForegroundColor Green
} else {
    Write-Host "[FAIL] IP not found on adapter" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "Testing connectivity to Jetson ($JetsonIP)..." -ForegroundColor Cyan
$ping = Test-Connection -ComputerName $JetsonIP -Count 2 -Quiet -ErrorAction SilentlyContinue
if ($ping) {
    Write-Host "[OK] Jetson is reachable at $JetsonIP" -ForegroundColor Green
} else {
    Write-Host "[WARN] Jetson at $JetsonIP did not respond to ping." -ForegroundColor Yellow
    Write-Host "       Make sure the Jetson is powered on and has IP 10.42.0.2 configured." -ForegroundColor Yellow
    Write-Host "       On the Jetson: sudo ip addr add 10.42.0.2/24 dev eth0 && sudo ip link set eth0 up" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "=== Done ===" -ForegroundColor Cyan
Write-Host "SSH from WSL:  ssh robotlab@$JetsonIP"
Write-Host "SSH shortcut:  ssh robotlab  (if ~/.ssh/config is set up)"
Write-Host ""
Write-Host "IMPORTANT: If this is the first time setting up mirrored networking,"
Write-Host "restart WSL from PowerShell:  wsl --shutdown"
Write-Host "Then reopen your WSL terminal."
