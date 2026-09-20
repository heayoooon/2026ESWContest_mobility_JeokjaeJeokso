param(
    [string]$RealityScan = 'C:\Program Files\Epic Games\RealityScan_2.2\RealityScan.exe',
    [string]$Workspace = 'C:\Users\DANHA\DigitalTwinWorkspace',
    [ValidateSet('Align','Mesh')][string]$Stage = 'Align'
)
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath $RealityScan)) { throw "RealityScan executable not found: $RealityScan" }
$output = Join-Path $Workspace 'reconstruction'
New-Item -ItemType Directory -Force -Path $output | Out-Null
$project = Join-Path $output 'cart.rsproj'
if ($Stage -eq 'Align') {
    if (Test-Path -LiteralPath $project) { throw "Project already exists: $project. Preserve it before a new alignment." }
    $first = Join-Path $Workspace 'frames\KakaoTalk_20260920_134934999'
    $second = Join-Path $Workspace 'frames\KakaoTalk_20260920_134931129'
    foreach ($folder in @($first,$second)) {
        if (!(Test-Path -LiteralPath $folder)) { throw "Missing frames: $folder" }
    }
    & $RealityScan -newScene -addFolder $first -addFolder $second -align -save $project
} else {
    if (!(Test-Path -LiteralPath $project)) { throw 'Run alignment and inspect the reconstruction region first.' }
    & $RealityScan -load $project -selectMaximalComponent -calculateNormalModel -save $project
}
if ($LASTEXITCODE -ne 0) { throw "RealityScan exited with code $LASTEXITCODE" }
