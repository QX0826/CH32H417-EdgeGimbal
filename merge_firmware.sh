#!/bin/bash
# ============================================================
# CH32H417 双核固件合并脚本
# 将 V3F.bin 和 V5F.bin 合并为一个可烧录的 Merge.bin
#
# 内存布局 (来自链接脚本):
#   V3F FLASH: 0x00000000, 64K
#   V5F FLASH: 0x00010000, 128K
#
# 烧录基地址: 0x00000000 (或 0x08000000 取决于烧录工具)
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
V3F_BIN="$SCRIPT_DIR/V3F/obj/CH32H417QEU_V3F.bin"
V5F_BIN="$SCRIPT_DIR/V5F/obj/CH32H417QEU_V5F.bin"
OUTPUT="$SCRIPT_DIR/V5F/obj/Merge.bin"

# V5F 在 Flash 中的偏移地址 (与链接脚本 Link_v5f.ld 中 FLASH ORIGIN 一致)
V5F_OFFSET=0x10000   # 65536 bytes

echo "============================================"
echo "  CH32H417 Dual-Core Firmware Merger"
echo "============================================"

# 检查文件是否存在
if [ ! -f "$V3F_BIN" ]; then
    echo "[ERROR] V3F binary not found: $V3F_BIN"
    echo "        Please build V3F project first!"
    exit 1
fi

if [ ! -f "$V5F_BIN" ]; then
    echo "[ERROR] V5F binary not found: $V5F_BIN"
    echo "        Please build V5F project first!"
    exit 1
fi

V3F_SIZE=$(stat -c%s "$V3F_BIN" 2>/dev/null || stat -f%z "$V3F_BIN")
V5F_SIZE=$(stat -c%s "$V5F_BIN" 2>/dev/null || stat -f%z "$V5F_BIN")

echo ""
echo "V3F binary: $V3F_BIN ($V3F_SIZE bytes)"
echo "V5F binary: $V5F_BIN ($V5F_SIZE bytes)"
echo "V5F offset: $V5F_OFFSET bytes"
echo ""

# 检查 V3F 是否超过 64K
if [ "$V3F_SIZE" -gt 65536 ]; then
    echo "[ERROR] V3F binary ($V3F_SIZE bytes) exceeds 64K flash region!"
    exit 1
fi

# 检查 V5F 是否超过 128K
V5F_MAX=$((128 * 1024))
if [ "$V5F_SIZE" -gt "$V5F_MAX" ]; then
    echo "[ERROR] V5F binary ($V5F_SIZE bytes) exceeds 128K flash region!"
    exit 1
fi

# 总大小 = V5F偏移 + V5F大小 (向上对齐到4字节)
TOTAL_SIZE=$((V5F_OFFSET + V5F_SIZE))
ALIGN4=$(( (TOTAL_SIZE + 3) & ~3 ))
TOTAL_SIZE=$ALIGN4

echo "Merge size: $TOTAL_SIZE bytes"

# 创建合并文件: 先用0xFF填充整个区域 (Flash擦除后默认值)
dd if=/dev/zero bs=1 count=$TOTAL_SIZE 2>/dev/null | tr '\0' '\377' > "$OUTPUT"

# 写入 V3F.bin 到偏移 0x0000
dd if="$V3F_BIN" of="$OUTPUT" bs=1 conv=notrunc 2>/dev/null
echo "[OK] V3F written at offset 0x00000000 ($V3F_SIZE bytes)"

# 写入 V5F.bin 到偏移 0x10000
dd if="$V5F_BIN" of="$OUTPUT" bs=1 seek=$V5F_OFFSET conv=notrunc 2>/dev/null
echo "[OK] V5F written at offset 0x00010000 ($V5F_SIZE bytes)"

echo ""
echo "============================================"
echo "  Merge.bin created: $OUTPUT"
echo "  Total size: $TOTAL_SIZE bytes"
echo "============================================"
echo ""
echo "Flashing instructions:"
echo "  1. Connect WCH-LinkE to the MCU"
echo "  2. Open WCH-LinkUtility (or MounRiver Studio)"
echo "  3. Flash Merge.bin to address 0x00000000"
echo "     (Some tools use 0x08000000 - try both if one fails)"
echo "  4. Reset the MCU"
echo ""
echo "Alternative: Flash separately"
echo "  - Flash V3F.bin to 0x00000000"
echo "  - Flash V5F.bin to 0x00010000"
