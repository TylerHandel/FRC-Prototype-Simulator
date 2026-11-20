# Downloads MuJoCo's release archive into third_party\mujoco (the version mujoco-rs binds), and
# copies mujoco.dll next to the debug and release builds so `cargo run` finds it.
# Run once after cloning, from the repository:  powershell -File tools\fetch-mujoco.ps1
$ErrorActionPreference = "Stop"
$Version = "3.17.0"
$Root = Split-Path -Parent $PSScriptRoot
$Dest = Join-Path $Root "third_party\mujoco"
$Zip = Join-Path $env:TEMP "mujoco-$Version-windows-x86_64.zip"
if (-not ((Test-Path "$Dest\VERSION") -and ((Get-Content "$Dest\VERSION") -eq $Version))) {
  Invoke-WebRequest -Uri "https://github.com/google-deepmind/mujoco/releases/download/$Version/mujoco-$Version-windows-x86_64.zip" -OutFile $Zip
  if (Test-Path $Dest) { Remove-Item -Recurse -Force $Dest }
  Expand-Archive -Path $Zip -DestinationPath $Dest
  Set-Content -Path "$Dest\VERSION" -Value $Version
}
foreach ($p in @("target\debug", "target\release", "target\debug\deps", "target\release\deps")) {
  $d = Join-Path $Root $p
  New-Item -ItemType Directory -Force -Path $d | Out-Null
  Copy-Item "$Dest\bin\mujoco.dll" $d -Force
}
Write-Host "MuJoCo $Version in $Dest"
