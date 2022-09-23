# Ask for Administrator permission first.
If (-NOT ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole] "Administrator")) {   
  $arguments = "& '" + $myinvocation.mycommand.definition + "'"
  Start-Process powershell -Verb runAs -ArgumentList $arguments
  Break
}

# Check Linux IP Address.
$linux_ip = bash.exe -c "ifconfig eth0 | grep 'inet '"
$found = $linux_ip -match '\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}';

# Check if we found a valid IP.
if( $found ){
  $linux_ip = $matches[0];
} else{
  Write-Output "The Script Exited, the ip address of WSL 2 cannot be found";
  exit;
}

# All the ports you want to forward separated by comma.
$ports=@(22, 11411);

# Windows IP Address for ipconfig to listen to.
$windows_ip='0.0.0.0';
$ports_a = $ports -join ",";

# Remove any existing Firewall exception rules.
Invoke-Expression "Remove-NetFireWallRule -DisplayName 'WSL 2 Firewall Unlock' ";

# Add new Firewall exception rules for inbound and outbound traffic.
Invoke-Expression "New-NetFireWallRule -DisplayName 'WSL 2 Firewall Unlock' -Direction Outbound -LocalPort $ports_a -Action Allow -Protocol TCP";
Invoke-Expression "New-NetFireWallRule -DisplayName 'WSL 2 Firewall Unlock' -Direction Inbound -LocalPort $ports_a -Action Allow -Protocol TCP";

# Map each listen port to the same port on Linux.
for( $i = 0; $i -lt $ports.length; $i++ ){
  $port = $ports[$i];
  Invoke-Expression "netsh interface portproxy delete v4tov4 listenport=$port listenaddress=$windows_ip";
  Invoke-Expression "netsh interface portproxy add v4tov4 listenport=$port listenaddress=$windows_ip connectport=$port connectaddress=$linux_ip";
}

# Cross check your work.
Invoke-Expression "netsh interface portproxy show v4tov4";