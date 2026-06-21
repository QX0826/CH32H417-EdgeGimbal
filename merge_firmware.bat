@echo off
chcp 65001 >nul
setlocal enabledelayedexpansion

:: ============================================================
:: CH32H417 双核固件合并脚本 (Windows)
:: 将 V3F.bin 和 V5F.bin 合并为一个可烧录的 Merge.bin
::
:: 内存布局 (来自链接脚本):
::   V3F FLASH: 0x00000000, 64K
::   V5F FLASH: 0x00010000, 128K
:: ============================================================

echo ============================================
echo   CH32H417 Dual-Core Firmware Merger
echo ============================================
echo.

set "SCRIPT_DIR=%~dp0"
set "V3F_BIN=%SCRIPT_DIR%V3F\obj\CH32H417QEU_V3F.bin"
set "V5F_BIN=%SCRIPT_DIR%V5F\obj\CH32H417QEU_V5F.bin"
set "OUTPUT=%SCRIPT_DIR%V5F\obj\Merge.bin"

:: V5F offset in bytes (0x10000 = 65536)
set /a V5F_OFFSET=65536

:: 检查文件
if not exist "%V3F_BIN%" (
    echo [ERROR] V3F binary not found: %V3F_BIN%
    echo         Please build V3F project first!
    pause
    exit /b 1
)

if not exist "%V5F_BIN%" (
    echo [ERROR] V5F binary not found: %V5F_BIN%
    echo         Please build V5F project first!
    pause
    exit /b 1
)

:: 获取文件大小
for %%A in ("%V3F_BIN%") do set V3F_SIZE=%%~zA
for %%A in ("%V5F_BIN%") do set V5F_SIZE=%%~zA

echo V3F binary: %V3F_BIN% (%V3F_SIZE% bytes)
echo V5F binary: %V5F_BIN% (%V5F_SIZE% bytes)
echo V5F offset: %V5F_OFFSET% bytes
echo.

:: 检查大小
if %V3F_SIZE% GTR 65536 (
    echo [ERROR] V3F binary exceeds 64K flash region!
    pause
    exit /b 1
)

set /a V5F_MAX=131072
if %V5F_SIZE% GTR %V5F_MAX% (
    echo [ERROR] V5F binary exceeds 128K flash region!
    pause
    exit /b 1
)

:: 计算总大小
set /a TOTAL_SIZE=V5F_OFFSET + V5F_SIZE
echo Merge size: %TOTAL_SIZE% bytes
echo.

:: 使用 PowerShell 创建合并文件
echo Creating merged binary...

powershell -NoProfile -Command ^
    "$v3f = [System.IO.File]::ReadAllBytes('%V3F_BIN%');" ^
    "$v5f = [System.IO.File]::ReadAllBytes('%V5F_BIN%');" ^
    "$total = %TOTAL_SIZE%;" ^
    "$merged = New-Object byte[] $total;" ^
    "for ($i = 0; $i -lt $total; $i++) { $merged[$i] = 0xFF };" ^
    "[System.Array]::Copy($v3f, 0, $merged, 0, $v3f.Length);" ^
    "[System.Array]::Copy($v5f, 0, $merged, %V5F_OFFSET%, $v5f.Length);" ^
    "[System.IO.File]::WriteAllBytes('%OUTPUT%', $merged);" ^
    "Write-Host '[OK] Merge.bin created successfully'"

if errorlevel 1 (
    echo [ERROR] Failed to create Merge.bin
    pause
    exit /b 1
)

echo.
echo ============================================
echo   Merge.bin: %OUTPUT%
echo   V3F at offset 0x00000000 (%V3F_SIZE% bytes)
echo   V5F at offset 0x00010000 (%V5F_SIZE% bytes)
echo   Total: %TOTAL_SIZE% bytes
echo ============================================
echo.
echo 烧录步骤:
echo   1. 连接 WCH-LinkE 到 MCU 的 SWD 接口
echo   2. 打开 WCH-LinkUtility 工具
echo   3. 选择芯片型号: CH32H417QEU
echo   4. 烧录 Merge.bin 到地址 0x00000000
echo      (如果报错，尝试地址 0x08000000)
echo   5. 复位 MCU
echo.
echo 或者分步烧录:
echo   - V3F.bin 烧录到 0x00000000
echo   - V5F.bin 烧录到 0x00010000
echo.
pause
