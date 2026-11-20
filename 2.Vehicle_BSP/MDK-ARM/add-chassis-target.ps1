# Add a CHASSIS_STEP target to Bench_Tests.uvprojx by cloning the WHEEL target,
# and register the two new source files (board_step6.c, chassis_step_smoke.c).
# Run with uVision CLOSED:
#   powershell -NoProfile -ExecutionPolicy Bypass -File "D:\Study\Next\32\Project\2.Vehicle_BSP\MDK-ARM\add-chassis-target.ps1"

$ErrorActionPreference = 'Stop'

$proj = 'D:\Study\Next\32\Project\2.Vehicle_BSP\MDK-ARM\Bench_Tests.uvprojx'
$backup = $proj + '.bak_before_chassis_step'

if (-not (Test-Path $proj)) { throw "project not found: $proj" }

$text = [System.IO.File]::ReadAllText($proj, [System.Text.Encoding]::UTF8)

if ($text -match '<TargetName>CHASSIS_STEP</TargetName>') {
    Write-Host "CHASSIS_STEP target already exists, nothing to do." -ForegroundColor Yellow
    exit 0
}

# ---------- 1. clone the WHEEL target block ----------
$nameTag = '<TargetName>WHEEL</TargetName>'
$nameIdx = $text.IndexOf($nameTag)
if ($nameIdx -lt 0) { throw "WHEEL target not found" }

$startIdx = $text.LastIndexOf('<Target>', $nameIdx)
if ($startIdx -lt 0) { throw "cannot find start of WHEEL target block" }

$endIdx = $text.IndexOf('</Target>', $nameIdx)
if ($endIdx -lt 0) { throw "cannot find end of WHEEL target block" }
$endIdx = $endIdx + '</Target>'.Length

$block = $text.Substring($startIdx, $endIdx - $startIdx)

$block = $block.Replace('<TargetName>WHEEL</TargetName>', '<TargetName>CHASSIS_STEP</TargetName>')
$block = $block.Replace('USE_HAL_DRIVER,STM32F407xx,WHEEL_TEST', 'USE_HAL_DRIVER,STM32F407xx,CHASSIS_STEP_TEST')
$block = $block.Replace('Bench_Tests\WHEEL\', 'Bench_Tests\CHASSIS_STEP\')
$block = $block.Replace('<OutputName>WHEEL</OutputName>', '<OutputName>CHASSIS_STEP</OutputName>')

if ($block -notmatch 'CHASSIS_STEP_TEST') { throw "define replacement failed" }

$insertAt = $text.IndexOf('</Targets>')
if ($insertAt -lt 0) { throw "</Targets> not found" }

$text = $text.Substring(0, $insertAt) + $block + "`r`n" + $text.Substring($insertAt)

# ---------- 2. register new source files ----------
function Add-FileEntry([string]$text, [string]$anchorFile, [string]$newName, [string]$newPath) {
    if ($text -match [regex]::Escape("<FileName>$newName</FileName>")) { return $text }

    $anchor = "<FileName>$anchorFile</FileName>"
    $idx = $text.IndexOf($anchor)
    if ($idx -lt 0) { throw "anchor file not found: $anchorFile" }

    # find the </File> that closes the anchor entry
    $closeIdx = $text.IndexOf('</File>', $idx)
    if ($closeIdx -lt 0) { throw "cannot find </File> after $anchorFile" }
    $closeIdx = $closeIdx + '</File>'.Length

    $entry = @"

            <File>
              <FileName>$newName</FileName>
              <FileType>1</FileType>
              <FilePath>$newPath</FilePath>
            </File>
"@
    return $text.Substring(0, $closeIdx) + $entry + $text.Substring($closeIdx)
}

$text = Add-FileEntry $text 'board_mpu6050.c' 'board_step6.c' '../BSP/Src/board_step6.c'
$text = Add-FileEntry $text 'rig_smoke.c' 'chassis_step_smoke.c' '../Tests/Src/chassis_step_smoke.c'

# ---------- 3. write back ----------
if (Test-Path $backup) { Remove-Item $backup -Force }
Copy-Item $proj $backup

$utf8NoBom = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText($proj, $text, $utf8NoBom)

Write-Host "OK: CHASSIS_STEP target added, backup at $backup" -ForegroundColor Green
Write-Host ("targets now: " + ([regex]::Matches($text, '<TargetName>').Count)) -ForegroundColor Green
