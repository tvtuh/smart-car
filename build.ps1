[CmdletBinding()]
param(
    [string]$Example = 'E07_09_wifi_spi_mt9v03x_demo_left_edge_fix',
    [string]$KeilRoot = $env:SMART_CAR_KEIL_ROOT,
    [switch]$List,
    [switch]$CheckOnly
)

$ErrorActionPreference = 'Stop'
$firmwareRoot = Join-Path $PSScriptRoot 'firmware\Motherboard_Demo'
$projectFiles = @(Get-ChildItem -LiteralPath $firmwareRoot -Recurse -File -Filter 'seekfree.uvproj' | Sort-Object FullName)
if ($projectFiles.Count -eq 0) {
    throw 'No Keil projects found. Download or clone the complete repository.'
}

if ($List) {
    foreach ($projectFile in $projectFiles) {
        $exampleDirectory = $projectFile.Directory.Parent.FullName
        Write-Output $exampleDirectory.Substring($firmwareRoot.Length + 1)
    }
    exit 0
}

# Check every project so a partial download fails before invoking the compiler.
$checkedReferences = 0
foreach ($projectFile in $projectFiles) {
    [xml]$projectXml = Get-Content -LiteralPath $projectFile.FullName -Raw
    $referenceNodes = @($projectXml.SelectNodes('//FilePath'))
    $includeNodes = @($projectXml.SelectNodes('//IncludePath'))
    $references = @($referenceNodes | ForEach-Object { $_.InnerText })
    foreach ($includeNode in $includeNodes) {
        $references += @($includeNode.InnerText -split ';' | Where-Object { $_ })
    }
    foreach ($reference in $references) {
        if ([IO.Path]::IsPathRooted($reference)) {
            throw "Non-portable absolute reference in $($projectFile.FullName): $reference"
        }
        $resolvedReference = [IO.Path]::GetFullPath((Join-Path $projectFile.Directory.FullName $reference))
        if (-not $resolvedReference.StartsWith($firmwareRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
            throw "Project reference points outside firmware: $reference"
        }
        if (-not (Test-Path -LiteralPath $resolvedReference)) {
            throw "Missing project dependency: $resolvedReference"
        }
        $checkedReferences++
    }
    if ($projectXml.SelectSingleNode('//RunUserProg1[text()="1"]') -or $projectXml.SelectSingleNode('//RunUserProg2[text()="1"]')) {
        throw "Review enabled external build hooks before using this helper: $($projectFile.FullName)"
    }
}
Write-Output "Dependency check: $($projectFiles.Count) projects, $checkedReferences file/include references found."
if ($CheckOnly) { exit 0 }

$selectedDirectory = [IO.Path]::GetFullPath((Join-Path $firmwareRoot $Example))
if (-not $selectedDirectory.StartsWith($firmwareRoot + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
    throw 'Example must be a directory inside firmware/Motherboard_Demo. Use -List.'
}
$selectedProject = Join-Path $selectedDirectory 'mdk\seekfree.uvproj'
if (-not (Test-Path -LiteralPath $selectedProject -PathType Leaf)) {
    throw "Example not found: $Example. Use -List to see available names."
}

if (-not $KeilRoot) {
    foreach ($candidateRoot in @('D:\Keil_v5', 'C:\Keil_v5', 'C:\Keil')) {
        if (Test-Path -LiteralPath (Join-Path $candidateRoot 'C251\BIN\C251.EXE')) {
            $KeilRoot = $candidateRoot
            break
        }
    }
}
if (-not $KeilRoot) {
    throw 'Keil C251 is required. Pass -KeilRoot C:\path\to\Keil or set SMART_CAR_KEIL_ROOT.'
}
$KeilRoot = [IO.Path]::GetFullPath($KeilRoot)
foreach ($requiredTool in @('UV4\UV4.exe', 'C251\BIN\C251.EXE', 'C251\BIN\A251.EXE', 'C251\BIN\L251.EXE', 'C251\BIN\OH251.EXE')) {
    if (-not (Test-Path -LiteralPath (Join-Path $KeilRoot $requiredTool) -PathType Leaf)) {
        throw "Missing Keil C251 component: $requiredTool under $KeilRoot"
    }
}

$logDirectory = Join-Path $PSScriptRoot '.build'
New-Item -ItemType Directory -Path $logDirectory -Force | Out-Null
$logName = ($Example -replace '[^a-zA-Z0-9_.-]', '_') + '.log'
$logFile = Join-Path $logDirectory $logName
$uvExecutable = Join-Path $KeilRoot 'UV4\UV4.exe'
$uvArguments = @('-r', ('"{0}"' -f $selectedProject), '-j0', '-o', ('"{0}"' -f $logFile))
Write-Output "Rebuilding: $Example"
$buildProcess = Start-Process -FilePath $uvExecutable -ArgumentList $uvArguments -WorkingDirectory (Join-Path $selectedDirectory 'mdk') -WindowStyle Hidden -PassThru
if (-not $buildProcess.WaitForExit(120000)) {
    $buildProcess.Kill()
    throw "Keil did not finish within 120 seconds. Check the toolchain and log: $logFile"
}
$buildProcess.WaitForExit()
if (-not (Test-Path -LiteralPath $logFile)) {
    throw "Keil returned without producing a build log. Check C251 installation: $logFile"
}
$buildLog = Get-Content -LiteralPath $logFile -Raw
$summary = [regex]::Match($buildLog, '(\d+)\s+Error\(s\),\s*(\d+)\s+Warning\(s\)', 'IgnoreCase')
if (-not $summary.Success -or [int]$summary.Groups[1].Value -ne 0 -or $buildProcess.ExitCode -gt 1) {
    Get-Content -LiteralPath $logFile -Tail 30 | Write-Output
    throw "Build failed. Full log: $logFile"
}
$hexFile = Join-Path $selectedDirectory 'mdk\out_file\SEEKFREE.hex'
if (-not (Test-Path -LiteralPath $hexFile -PathType Leaf)) {
    throw "Compiler reported no errors but HEX was not found: $hexFile"
}
Write-Output "Build finished: $($summary.Groups[1].Value) errors, $($summary.Groups[2].Value) warnings."
Write-Output "Firmware: $hexFile"
Write-Output "Build log: $logFile"
Write-Output 'No hardware has been flashed. Verify wiring and local configuration before flashing.'
exit 0
