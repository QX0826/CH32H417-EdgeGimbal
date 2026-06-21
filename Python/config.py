"""
CH32H417 云台控制系统 - Python 上位机配置
==========================================
只修改这个文件中的 COM 口号，其他代码不用动。

接线对应关系：
  COM口1 (9600bps)  → MCU USART8 (PE14/PE15) → 手势指令
  COM口2 (115200bps) → MCU USART6 (PE8/PE9)   → 人脸坐标
  COM口3 (115200bps) → MCU USART3 (PB10/PB11) → 语音触发/调试
"""

# ===== 在这里修改 COM 口号 =====
# 插上 USB-UART 适配器后，在设备管理器中查看分配的 COM 号
GESTURE_COM_PORT = 'COM22'    # 连接到 USART8 (手势指令)
FACE_COM_PORT    = 'COM20'    # 连接到 USART6 (人脸坐标)
MCU_COM_PORT     = 'COM9'     # 连接到 USART3 (语音触发/调试)

# ===== 波特率（一般不用改） =====
GESTURE_BAUDRATE = 9600       # 与 USART8 匹配
FACE_BAUDRATE    = 115200     # 与 USART6 匹配
MCU_BAUDRATE     = 115200     # 与 USART3 匹配
