# Disable Hyper-V Firewall for WSL2
# Run this from an ELEVATED PowerShell (Run as Administrator)
#
# This sets the Hyper-V VM firewall (which WSL2 uses) to allow all inbound traffic.
# The regular Windows Firewall still protects the host machine.

Write-Host "=== Disable Hyper-V Firewall for WSL2 ===" -ForegroundColor Cyan

# Get the WSL2 Hyper-V firewall settings
$vmId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'

try {
    $settings = Get-NetFirewallHyperVVMSetting -Name $vmId -ErrorAction Stop
    Write-Host "Current settings:"
    Write-Host "  Enabled: $($settings.Enabled)"
    Write-Host "  DefaultInboundAction: $($settings.DefaultInboundAction)"
    Write-Host "  DefaultOutboundAction: $($settings.DefaultOutboundAction)"
} catch {
    Write-Host "ERROR: Could not read Hyper-V firewall settings." -ForegroundColor Red
    Write-Host "Make sure you are running PowerShell as Administrator." -ForegroundColor Yellow
    exit 1
}

# Set default inbound to Allow
Set-NetFirewallHyperVVMSetting -Name $vmId -DefaultInboundAction Allow

# Verify
$settings = Get-NetFirewallHyperVVMSetting -Name $vmId
Write-Host ""
Write-Host "Updated settings:"
Write-Host "  Enabled: $($settings.Enabled)"
Write-Host "  DefaultInboundAction: $($settings.DefaultInboundAction)" -ForegroundColor Green
Write-Host ""
Write-Host "Done. The Hyper-V firewall now allows inbound traffic to WSL2." -ForegroundColor Green
Write-Host "The regular Windows Firewall still protects the host." -ForegroundColor Yellow
