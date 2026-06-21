/********************************** (C) COPYRIGHT ******************************
 * File Name          : shared_gimbal.c
 * Description        : 双核共享变量定义
 *                      放在 .shared_data 段，V5F和V3F均可访问
 *                      参考: ch32h417-main/EVT/EXAM/CPU/HSEM/HSEM_DataSharing/V3F/User/shared.c
 ******************************************************************************
 */

#include "stdio.h"
#include "shared_gimbal.h"

/* 共享数据 — 放在链接脚本定义的 .shared_data 段 */
__attribute__((section(".shared_data")))
volatile GimbalSharedData_t gimbal_shared = {0};
