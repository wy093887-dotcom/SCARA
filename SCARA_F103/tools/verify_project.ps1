param(
    [string]$Preset = "Debug"
)

# 项目自检脚本：构建固件、检查产物、确认课程设计版保留的脚本和文档存在。

$ErrorActionPreference = "Stop"

$projectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
$buildDir = Join-Path $projectRoot "build\$Preset"
$firmwareBase = Join-Path $buildDir "SCARA_F103"
$failures = 0

function Pass {
    param([string]$Message)
    Write-Host "PASS $Message"
}

function Fail {
    param([string]$Message)
    Write-Host "FAIL $Message"
    $script:failures++
}

function Require-File {
    param([string]$Path)
    if (Test-Path $Path -PathType Leaf) {
        Pass "file exists: $Path"
    } else {
        Fail "missing file: $Path"
    }
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Description
    )

    if (-not (Test-Path $Path -PathType Leaf)) {
        Fail "cannot check missing file: $Path"
        return
    }

    $text = Get-Content -Raw $Path
    if ($text -match $Pattern) {
        Pass $Description
    } else {
        Fail $Description
    }
}

function Require-DocFile {
    param(
        [string]$RelativePath,
        [string]$AbsoluteFallback
    )

    $relative = Join-Path $projectRoot $RelativePath
    if (Test-Path $relative -PathType Leaf) {
        Pass "file exists: $relative"
        return
    }

    if (Test-Path $AbsoluteFallback -PathType Leaf) {
        Pass "file exists: $AbsoluteFallback"
        return
    }

    Fail "missing file: $relative"
}

Write-Host "== Build =="
Push-Location $projectRoot
try {
    & cmake --build --preset $Preset
    if ($LASTEXITCODE -ne 0) {
        Fail "cmake build preset $Preset"
    } else {
        Pass "cmake build preset $Preset"
    }
} finally {
    Pop-Location
}

Write-Host "== Firmware Artifacts =="
Require-File "$firmwareBase.elf"
Require-File "$firmwareBase.hex"
Require-File "$firmwareBase.bin"
Require-File "$firmwareBase.map"

$binPath = "$firmwareBase.bin"
if (Test-Path $binPath -PathType Leaf) {
    $binSize = (Get-Item $binPath).Length
    $limit = 63 * 1024
    if ($binSize -le $limit) {
        Pass "binary size $binSize <= $limit bytes"
    } else {
        Fail "binary size $binSize > $limit bytes"
    }
}

Write-Host "== Flash Layout =="
$linker = Join-Path $projectRoot "STM32F103XX_FLASH.ld"
$config = Join-Path $projectRoot "UserApp\app_config.h"
Require-Text $linker "FLASH \(rx\)\s+:\s+ORIGIN = 0x8000000, LENGTH = 63K" "linker reserves final parameter page"
Require-Text $config "APP_PARAM_FLASH_ADDR 0x0800F800u" "parameter page address is 0x0800F800"
Require-Text $config "APP_FW_VERSION `"0\.23\.1`"" "firmware version is 0.23.1"
Require-Text $config "APP_SERIAL_BAUDRATE 115200u" "serial baudrate is 115200"
Require-Text $config "APP_COMM_WATCHDOG_DEFAULT_MS 0u" "comm watchdog is disabled by default"
Require-Text $config "APP_HOST_OWNS_LIMIT_CHECKS 1u" "host owns trajectory limit checks"

Write-Host "== Build Rules and VS Code =="
$cmake = Join-Path $projectRoot "CMakeLists.txt"
Require-Text $cmake "-O ihex" "CMake generates hex"
Require-Text $cmake "-O binary" "CMake generates bin"
Require-Text $cmake "UserApp/gcode_stream\.c" "CMake builds gcode stream"
Require-Text $cmake "UserApp/home_controller\.c" "CMake builds home controller"
Require-Text $cmake "UserApp/home_sensor\.c" "CMake builds home sensor"
Require-Text $cmake "(?s)^(?!.*UserApp/pulse_protocol\.c)" "CMake does not build old pulse protocol"
Require-Text $cmake "(?s)^(?!.*UserApp/trajectory\.c)" "CMake does not build old trajectory queue"
Require-Text $cmake "(?s)^(?!.*UserApp/teach\.c)" "CMake does not build old teach module"
Require-File (Join-Path $projectRoot ".vscode\tasks.json")
Require-File (Join-Path $projectRoot ".vscode\launch.json")
Require-File (Join-Path $projectRoot ".vscode\settings.json")
Require-File (Join-Path $projectRoot "Run_Serial_Test.bat")
Require-File (Join-Path $projectRoot "Run_COM13_HostPlanned_3000.bat")
Require-File (Join-Path $projectRoot "tools\serial_link_check.ps1")
Require-File (Join-Path $projectRoot "tools\home_sensor_check.ps1")
Require-File (Join-Path $projectRoot "tools\gcode_stream_check.ps1")
Require-File (Join-Path $projectRoot "tools\host_planned_stream_stress.ps1")
Require-File (Join-Path $projectRoot "tools\robot_upper_sim\planner_core.py")
Require-File (Join-Path $projectRoot "tools\robot_upper_sim\upper_sim.py")
Require-File (Join-Path $projectRoot "tools\robot_upper_sim\README.md")
Require-File (Join-Path $projectRoot "tools\robot_upper_sim\Run_Robot_UpperSim.bat")

Write-Host "== Documentation =="
Require-DocFile "..\Version.md" "C:\Users\22602\Desktop\SCARA\Version.md"
Require-DocFile "..\Control.md" "C:\Users\22602\Desktop\SCARA\Control.md"
Require-DocFile "..\Work.md" "C:\Users\22602\Desktop\SCARA\Work.md"

if ($failures -eq 0) {
    Write-Host "VERIFY PASS"
    exit 0
}

Write-Host "VERIFY FAIL failures=$failures"
exit 1
