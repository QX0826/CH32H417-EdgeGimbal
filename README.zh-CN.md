# CH32H417-EdgeGimbal

🚀 **基于 CH32H417 与 TFLM 框架开发的分布式异构协同边缘 AI 智能云台系统**

<p align="center">
  <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License">
  <img src="https://img.shields.io/badge/MCU-CH32H417QEU6-orange.svg" alt="MCU">
  <img src="https://img.shields.io/badge/AI--Framework-TFLM--TensorFlow-green.svg" alt="Framework">
  <img src="https://img.shields.io/badge/RealTime-30FPS-brightgreen.svg" alt="FrameRate">
</p>

`CH32H417-EdgeGimbal` 是一款面向物联网、机器人和智能终端的**分级异构边缘 AI 稳像云台系统**。系统完全脱离云端运行，具备低延迟、高安全性及低功耗等端侧核心优势，实现人脸追踪、手势交互和离线语音命令的多模态联动控制。

---

## 📖 项目概述 (Overview)

本项目采用“**边缘视觉网关 + 离线语音芯片 + 端侧智能决策中心**”的协同计算架构。高算力端（PC 或边缘伴侣）运行基于神经网络的人脸追踪与手势识别算法，ASRPRO 芯片处理离线人声指令，而主频 400MHz 的 CH32H417 则作为多模态数据融合与精密动作执行的核心中枢。

---

## ⚡ 核心特性 (Features)

* **🧠 TFLM 端侧推理决策**：基于 TensorFlow Lite for Microcontrollers (TFLM) 框架在 MCU 端部署轻量决策模型，执行手势及状态的本地分类。
* **⏳ 时间预算管理**：引入端侧时间预算（Time Budget）算法，实现硬实时控制任务与端侧 AI 推理任务的时间片隔离，保证舵机伺服周期的确定性。
* **📐 分级异构协同**：将高计算密集的视觉模型与微秒级硬实时运动算法解耦在不同算力平台上运行，最大化系统运行效能。
* **📈 运动状态估计与自适应频率**：MCU 运行运动状态前馈估计，在目标静止时动态降低检测频次，可降低边缘视觉端 66% 的推理功耗。
* **🌀 两级 EMA 滤波与死区机制**：采用输入/输出双重指数移动平均（EMA）滤波，配合双轴独立迟滞死区，完美滤除视觉像素级抖动。
* **⚡ 三段式速度收敛算法**：根据角度误差动态切换“二分追赶”、“三分减速”与“符号函数微调”，天然无过冲风险。
* **🔄 双核迁移友好设计**：设计了读写解耦的全局数据共享结构体 `gimbal_shared`，完美适配 CH32H417 双核异构任务迁移。

---

## 🛠️ 技术栈 (Tech Stack)

| 维度 | 技术/框架/硬件 | 作用/说明 |
|:---|:---|:---|
| **主控芯片** | WCH CH32H417QEU6 | 400MHz 高性能 RISC-V 内核，提供端侧算力支持 |
| **下位机软件库**| Standard Peripheral Library | 官方标准外设库，驱动硬件 TIM、USART、DMA |
| **端侧 AI 框架**| TensorFlow Lite Micro (TFLM) | 负责下位机端侧手势分类与状态决策推理 |
| **视觉推理库** | InsightFace & MediaPipe | 负责边缘端的高精度人脸追踪与手势骨架回归 |
| **视觉网关** | Python 3.9 + OpenCV + PySerial | 负责视频流采集、第一级平滑滤波及高频串口通信 |
| **声学交互** | ASRPRO-01 | 搭载神经网络的离线语音芯片，负责语音命令解码 |

---

## 📂 项目结构 (Project Structure)

```text
├── Common/              # 双核公共驱动、寄存器定义与外设库 (SPL)
│   └── Common/          # 共享内存通信接口 (shared_gimbal)
├── Python/              # 上位机边缘视觉网关程序
│   ├── config.py        # 限幅、死区与滤波参数配置
│   └── 识别.py          # 视觉分析与数据发送主入口
├── V3F/                 # MCU V3F 核心工程（实时控制）
│   └── User/            # 舵机驱动与三段式速度收敛算法 (servo.c)
├── V5F/                 # MCU V5F 核心工程（状态决策）
│   ├── App/             # TFLM 推理决策层
│   └── User/            # 多模态状态机交互逻辑 (main.c)
├── CH32H417QEU.wvsln    # MounRiver Studio 解决方案文件
├── merge_firmware.bat   # 双核 Hex 固件快速合并脚本 (Windows)
├── merge_firmware.sh    # 双核 Hex 固件快速合并脚本 (Linux)
└── 语音控制.hd          # 离线语音词条配置文件
```

---

## 🔌 硬件接线指南 (Hardware Connections)

> [!IMPORTANT]
> 舵机工作瞬间电流大，必须由独立外部 5V-6V 电源供电，且外部电源负极必须与开发板的 **GND** 连通实现共地。

| 外设接口 | 单片机引脚 | 连接对象 | 说明 | 默认波特率 |
|:---|:---|:---|:---|:---:|
| **USART8** | PE7 (RX) / PE8 (TX) | 电脑 USB-to-UART | 主通信与状态握手通道 | 115200 bps |
| **USART3** | PB11 (RX) / PB10 (TX)| 电脑 USB-to-UART | 高频人脸追踪坐标通道 | 115200 bps |
| **USART4** | PC7 (RX) / PC6 (TX) | 电脑 USB-to-UART | 手势识别指令接收通道 | 9600 bps |
| **USART2** | PD6 (RX) / PD5 (TX) | 语音模块 TX1/RX1 | 语音控制指令接收通道 | 9600 bps |
| **USART1** | PA10 (RX) / PA9 (TX) | 语音模块 TX2/RX2 | 语音播报反馈发送通道 | 19200 bps |
| **TIM2_CH1**| PA0 | 俯仰 (Pitch) 舵机 | 输出 50Hz PWM 控制信号 | — |
| **TIM2_CH2**| PA1 | 偏航 (Yaw) 舵机 | 输出 50Hz PWM 控制信号 | — |

---

## 🚀 快速开始 (Getting Started)

### 1. 编译并烧录下位机固件
1. 使用 **MounRiver Studio** 打开 `CH32H417QEU.wvsln`。
2. 分别构建 `V3F` 与 `V5F` 工程以生成对应的 Hex 文件。
3. 双击运行 `merge_firmware.bat`，合并双核固件并使用 WCH-Link 进行烧录。

### 2. 运行上位机视觉程序
安装依赖：
```bash
pip install opencv-python numpy pyserial mediapipe insightface onnxruntime
```
修改 `Python/识别.py` 底部对应的串口号，然后启动：
```bash
python Python/识别.py
```

---

## 🔒 安全与隐私 (Security & Privacy)

本项目的下位机及上位机配置仅提供结构演示。生产部署时的模型私钥、设备凭据和通信密钥应当通过环境变量或私有配置文件进行注入，请勿将其硬编码提交至代码仓库。涉及图像捕获的调试阶段，请优先使用脱敏或虚拟数据。

---

## 📄 开源协议 (License)

本项目采用 [MIT License](LICENSE) 许可证进行开源。
