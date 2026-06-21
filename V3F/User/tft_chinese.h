/********************************** (C) COPYRIGHT ******************************
 * File Name          : tft_chinese.h
 * Description        : TFT180 中文16x16字模数据
 *                      使用 tft180_show_chinese() 显示
 *                      每个汉字 16x16 点阵，16字节/字
 ******************************************************************************
 */
#ifndef __TFT_CHINESE_H
#define __TFT_CHINESE_H

#include "ch32h417.h"

extern const uint8_t tft_chinese[][16];

/* 字模索引 (按原始项目顺序, 每字2个数组元素 = 32字节) */
#define CH_HUAN_YING       0   /* 欢迎 */
#define CH_WEN_DING        2   /* 稳定 */
#define CH_YUN_TAI         4   /* 云台 */
#define CH_DANG_QIAN       8   /* 当前 */
#define CH_WEN_DU         12   /* 温度 */
#define CH_SHI_DU         16   /* 湿度 */
#define CH_QI_TI           20   /* 气体 */
#define CH_JU_LI           24   /* 距离 */
#define CH_GONG_YING       28   /* 供应 */
#define CH_YAN_WU          32   /* 烟雾 */
#define CH_NONG_DU         36   /* 浓度 */
#define CH_SHE_DING        40   /* 设定 */
#define CH_JIAN_CE         44   /* 检测 */
#define CH_HUO_YAN         48   /* 火焰 */
#define CH_BAO_JING        52   /* 报警 */
#define CH_WU              56   /* 无 */
#define CH_YOU             60   /* 有 */
#define CH_ANG             64   /* 盎 */
#define CH_JING            68   /* 警 */
#define CH_WEI             72   /* 未 */
#define CH_DAO             76   /* 到 */
#define CH_CHAO            80   /* 超 */
#define CH_BIAO            84   /* 标 */
#define CH_TIAN_RAN_QI     88   /* 天然气 */
#define CH_DENG            92   /* 等 */
#define CH_JI              96   /* 级 */
#define CH_MAO_XIAN       100   /* 冒险 */
#define CH_XIAN           104   /* 险 */
#define CH_ZHENG_CHANG    108   /* 正常 */
#define CH_XIAO_FANG      112   /* 消防 */
#define CH_KE_JI          116   /* 科技 */
#define CH_JIAN_KANG      120   /* 健康 */
#define CH_BAO_HU         124   /* 保护 */
#define CH_XI_TONG        128   /* 系统 */
#define CH_QI_DONG        132   /* 启动 */
#define CH_ZHONG          136   /* 中 */

#endif /* __TFT_CHINESE_H */
