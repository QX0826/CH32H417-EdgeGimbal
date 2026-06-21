# CH32H417-EdgeGimbal

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/MCU-CH32H417-orange.svg)](http://www.wch.cn)
[![Framework](https://img.shields.io/badge/AI-TFLM-green.svg)](https://www.tensorflow.org/lite/microcontrollers)

A distributed heterogeneous Edge AI gimbal stabilization system powered by CH32H417 and TensorFlow Lite for Microcontrollers (TFLM).  
基于 CH32H417 与 TFLM 框架开发的分布式异构协同边缘 AI 智能云台系统。

---

## 📖 项目简介 (Introduction)

`CH32H417-EdgeGimbal` 是一款面向物联网与智能终端的**分级异构边缘 AI 稳像云台系统**。系统完全脱离云端运行，具备低延迟、高安全性及低功耗等端侧核心优势。

项目采用“**边缘视觉网关 + 离线语音芯片 + 端侧智能决策中心**”的协同计算架构：
1. **视觉网关端 (PC/边缘板卡)**：执行人脸实时追踪与 MediaPipe 21点手势骨架回归。
2. **语音声学端 (ASRPRO)**：处理离线人声指令，实现即时控制。
3. **端侧决策中心 (CH32H417, 400MHz RISC-V)**：运行多模态融合状态机，并通过部署的 **TFLM 决策分类模型** 与**时间预算管理算法**，实现毫秒级的时间片调度，确保硬实时舵机控制周期的确定性。

---

## ⚡ 核心算法与特性 (Key Features)

* **TFLM 端侧推理与时间预算调度**：
  在 CH32H417 核心上基于 **TensorFlow Lite for Microcontrollers (TFLM)** 部署轻量化决策/分类模型。引入**端侧时间预算管理算法**，实现硬实时控制任务与端侧 AI 推理任务的毫秒级时间片隔离，保证舵机伺服周期的确定性。
* **两级 EMA 滤波与自适应迟滞死区**：
  设计输入/输出双重指数移动平均（EMA）滤波，配合双轴独立迟滞死区，完美滤除视觉检测的像素级抖动与噪声，避免舵机在微小偏差下的频繁“寻优”耗电，大幅提升寿命。
* **三段式误差自适应速度算法**：
  下位机采用高效的整数自适应速度决策：大误差快速赶超、中误差减速靠拢、小误差微调落座，天然无过冲风险，运行流畅。
* **自适应检测频率与运动状态估计**：
  MCU 端运行等速运动前馈估计，当目标慢速运动或静止时动态降频，**降低边缘视觉端 66% 的推理功耗**；在检测间隔帧进行轨迹线性外推，确保 30FPS 级别的连续伺服稳定性。
* **数据解耦与多核迁移就绪**：
  设计了读写解耦的全局数据共享结构体 `gimbal_shared`，完美适配 CH32H417（V5F + V3F）双核异构任务迁移。

---

## 📂 项目结构 (Repository Structure)

```text
├── Common/              # 公共外设驱动与异构数据共享头文件
├── Python/              # 上位机视觉网关脚本（人脸/手势追踪）
├── V3F/                 # CH32H417 V3F 核心工程（实时伺服驱动）
├── V5F/                 # CH32H417 V5F 核心工程（多模态融合与TFLM模型部署）
├── CH32H417QEU.wvsln    # MounRiver Studio 解决方案文件
├── merge_firmware.bat   # 固件合并脚本 (Windows)
├── merge_firmware.sh    # 固件合并脚本 (Linux)
├── 识别.py              # 上位机主入口脚本
└── 语音控制.hd          # 离线语音指令配置
```

---

## 🚀 快速开始 (Getting Started)

### 1. 硬件连接
* **主控芯片**：CH32H417QEU6 开发板。
* **执行机构**：TIM2（PA0, PA1）驱动两轴 PWM 舵机。
* **语音模块**：ASRPRO-01 离线语音模块通过 USART2 (PD5, PD6) 连接。
* **通信接口**：上位机通过 USB-to-UART 模块分别连接 USART3 (PB10/PB11, 图像坐标)、USART4 (PC6/PC7, 手势指令) 以及 USART8 (PE7/PE8, 主通信)。

### 2. 软件配置
#### 上位机环境
请确保安装 Python 3.8+ 并配置以下依赖：
```bash
pip install opencv-python numpy pyserial mediapipe insightface onnxruntime
```
修改 `识别.py` 中的串口号配置，然后运行：
```bash
python 识别.py
```

#### 下位机工程
1. 使用 **MounRiver Studio** 打开 `CH32H417QEU.wvsln` 解决方案。
2. 编译 `V3F` 和 `V5F` 工程。
3. 可使用固件合并脚本合并后烧录至 CH32H417 芯片中。

---

## 📄 开源协议 (License)

本项目采用 [MIT License](LICENSE) 开源协议。
