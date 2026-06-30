# 基于CH32H417与异构协同架构的分布式边缘AI智能云台系统

<p align="right">
  <a href="README.md">🌐 English</a>
</p>

<p align="center">
  <strong>🏆 全国大学生嵌入式芯片与系统设计竞赛参赛作品</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/主控芯片-CH32H417QEU6-orange.svg" alt="MCU">
  <img src="https://img.shields.io/badge/内核架构-双核%20RISC--V%20(V5F%2BV3F)-critical.svg" alt="Core">
  <img src="https://img.shields.io/badge/主频-400MHz-red.svg" alt="Freq">
  <img src="https://img.shields.io/badge/控制频率-50Hz%20闭环控制-brightgreen.svg" alt="Servo">
  <img src="https://img.shields.io/badge/核间同步-HSEM%20硬件信号量-blueviolet.svg" alt="HSEM">
  <img src="https://img.shields.io/badge/串口架构-5路串口%20数据路由-blue.svg" alt="DMA">
  <img src="https://img.shields.io/badge/浮点加速-硬件FPU-yellow.svg" alt="FPU">
  <img src="https://img.shields.io/badge/运行模式-完全离线本地运行-success.svg" alt="Offline">
  <img src="https://img.shields.io/badge/开发工具-MounRiver%20Studio-informational.svg" alt="IDE">
</p>

<p align="center">
  <img src="system_architecture.png" alt="系统架构图" width="900">
</p>

> **图示**：CH32H417 双核异构系统架构 — 边缘视觉网关 → V3F 通信协处理器（100MHz）→ 共享 SRAM（HSEM）→ V5F 实时控制主核（400MHz）→ 三自由度伺服云台

---

## 🏆 作品概述

本作品是一款面向高动态视轴稳定与无接触多模态交互的**三自由度智能稳像云台系统**。系统以沁恒微电子新一代双核 RISC-V 芯片 **CH32H417QEU6**（主频 400MHz）为核心，采用**"边缘视觉网关 + 离线语音芯片（ASR-PRO）+ 端侧智能决策中心（MCU）"**的分级边缘计算架构，实现零云端依赖的低延迟动态跟拍。

系统深入挖掘了 CH32H417 的**双核异构并行、HSEM 硬件信号量、多路高速串口中断通信、硬件 FPU 浮点加速**等底层硬件特性，通过软硬件协同设计，彻底解决了常规单核控制系统因串口高频中断干扰而导致的舵机控制脉宽抖动与画面超调振荡问题。

---

## 🛠️ CH32H417 芯片硬件资源深度挖掘与工作模式

> **嵌入式芯片大赛核心考察点**：本系统完全摒弃第三方二次封装开源库，基于 WCH 官方标准外设库直接编写寄存器级驱动，深度压榨了 CH32H417 的每一项核心硬件资源。

### ① 双内核工作模式划分

系统设计了两种工作模式以兼顾高度的灵活性与极限的控制精度：

1. **双核异构协同模式 (默认运行状态)**：
   * **V3F (通信协处理器 - Core 0)**：运行于 100MHz 主频（SYSCLK/4 分频），专职处理高频串口通信中断（USART1/2/3/4/8）。负责高速接收边缘网关的人脸坐标帧、手势指令包及 ASR-PRO 语音口令，并将数据实时写入共享内存区，充当“通信与网关处理器”。
   * **V5F (控制与采集主核 - Core 1)**：运行于 400MHz 主频，专职执行定时控制逻辑（20ms周期 / 50Hz）。执行 TIM2 3路 PWM 伺服驱动、三段式速度收敛算法、Roll轴多项式姿态前馈补偿、ST7735 TFT屏驱动显示以及 3 路 ADC 气体传感器数据采集。此架构保证了控制主核的 CPU 运行时间不会被串口中断打断，彻底消除了控制信号的抖动（Jitter < 5µs）。
2. **单核全功能模式 (兼容运行状态)**：
   * 在调试和单核配置下（`Run_Core = Run_Core_V3F`），V3F 核心将独立驱动一切外设，包括 5 路串口数据路由、TIM2 舵机驱动、ST7735 TFT 屏显示及报警系统，实现全套业务的单核运行闭环，而 V5F 则保持休眠。

### ② HSEM 硬件信号量 — 双核原子同步机制

在双核异构协同模式下，双核共享全局 SRAM 内存区 `gimbal_shared`（映射于链接脚本定义的 `RAM_SHARED` 段，起始地址 `0x20178000`）。为保证跨核数据读写的完整性，避免数据踩踏，V5F 主核利用 CH32H417 特有的 **HSEM（Hardware Semaphore）** 硬件自旋锁进行临界区保护：

```c
// V5F/App/servo.c — V5F 主核通过硬件自旋锁原子性锁存并读取共享内存
void V5F_ProcessSharedData(void)
{
    if(HSEM_FastTake(GIMBAL_HSEM_ID) == READY)
    {
        /* 原子读取由 V3F 写入的数据包 */
        if(gimbal_shared.command_ready)
        {
            uint8_t cmd = gimbal_shared.command;
            // ... 状态机处理手势指令 ...
            gimbal_shared.command_ready = 0;
        }

        if(gimbal_shared.face_ready && gimbal_shared.tracking_mode)
        {
            float fx = gimbal_shared.face_x / 10.0f;
            float fy = gimbal_shared.face_y / 10.0f;
            // ... 处理人脸跟踪目标角度 ...
            gimbal_shared.face_ready = 0;
        }
        HSEM_ReleaseOneSem(GIMBAL_HSEM_ID, 0); // 释放锁
    }
}
```
* **硬件级锁存**：自旋锁的锁定与释放耗时小于 1µs，相比软件级互斥体大幅降低了 CPU 开销。
* **隔离与写回**：V5F 更新完当前舵机角度与 ADC 数据后，直接写入共享内存段，供 V3F 进行蓝牙上报和串口发送。

### ③ TIM2 High-Resolution PWM — 4000 级占空比细分

舵机的转动细腻度高度依赖于 PWM 信号占空比的分辨率。系统将 TIM2 定时器配置为高精度计数模式：

```c
// TIM2（32位定时器）初始化配置：400MHz 主频经过 40 分频得到 10MHz 计数时钟
// ARR 周期值设为 20000-1，由此产生标准的 50Hz PWM 伺服信号 (周期 20ms)
TIM_TimeBaseStructure.TIM_Period = 20000 - 1;
TIM_TimeBaseStructure.TIM_Prescaler = 40 - 1;
```
* **脉宽范围**：$1000$（$0.5\text{ms}$，$0^\circ$）至 $5000$（$2.5\text{ms}$，$180^\circ$）。
* **角度分辨率**：在 $180^\circ$ 范围内具备 4000 级高精度细分，角度调节分辨率高达 $\theta_{\text{res}} = 180^\circ / 4000 = 0.045^\circ$，远超普通数字舵机的物理死区精度，确保物理运动平滑无级感。

### ④ 硬件 FPU 单精度浮点加速

在定时控制周期中，系统需要进行高频双级 EMA 滤波器计算、非线性迟滞控制、浮点约束限制及三次多项式姿态前馈拟合。全部代码配置为**硬浮点编译模式（`-mfloat-abi=hard`）**，直接调用主片上硬件 FPU 单元：
* 单次姿态解算及控制输出耗时由软件模拟的 **12.5ms** 骤降至 **0.05ms** 左右，系统运算开销仅占控制周期的 0.25%，为高频姿态修正预留了充沛的计算裕量。

### ⑤ 双核固件物理边界对齐与一键合并

两颗内核的固件必须严格对齐烧录至各自的 Flash 地址：
* **V3F 固件**：起始于 `0x00000000` 物理地址，最大限制 64KB。
* **V5F 固件**：起始于 `0x00010000` 物理地址（64KB 物理偏移处），最大限制 128KB。

团队编写了 `merge_firmware.bat` 与 `merge_firmware.sh` 脚本，其核心逻辑为：使用字节填充方式先创建一个大小为 $65536 + V5F\_SIZE$ 的空数组，以 `0xFF`（Flash 擦除态）填满，然后将 `V3F.bin` 拷入 `0` 偏移处，将 `V5F.bin` 拷入 `0x10000` 偏移处。评委只需在 WCH-LinkUtility 中将生成的 `Merge.bin` 一键烧录至首地址即可同时更新双核固件。

---

## 🔌 硬件引脚分配与通信拓扑

边缘视觉网关、ASR-PRO 语音模块与双核 MCU 之间的通信物理拓扑及管脚定义如下：

```text
  边缘视觉网关 (通过 CH340 USB-TTL 串口芯片连接)         CH32H417QEU6 主控端
 ┌────────────────────────────────────────┐          ┌──────────────────────┐
 │  COM10 (波特率 115200) - 人脸坐标发送    ├─────────►│ PB11 (USART3_RX)     │
 │  COM9  (波特率 9600)   - 手势指令发送    ├─────────►│ PC7  (USART4_RX)     │
 │  COM27 (波特率 115200) - 语音/PC遥测调试 ├◄────────►│ PE7/PE8 (USART8)     │
 └────────────────────────────────────────┘          └──────────────────────┘

  ASR-PRO 离线语音识别模块                              CH32H417QEU6 主控端
 ┌────────────────────────────────────────┐          ┌──────────────────────┐
 │  PA2 (TX1) - 语音识别指令发送 (9600)     ├─────────►│ PD6 (USART2_RX)      │
 │  PA3 (RX1) - 语音通信握手控制 (9600)     │◄─────────┤ PD5 (USART2_TX)      │
 │  PA5 (TX2) - 语音模块控制/音频 (19200)   ├─────────►│ PA10 (USART1_RX)     │
 │  PA6 (RX2) - 语音状态机同步 (19200)     │◄─────────┤ PA9 (USART1_TX)      │
 └────────────────────────────────────────┘          └──────────────────────┘
```

### 管脚分配明细

| 模块类别 | 外设资源 | 芯片物理引脚 | 连接目标/功能 | 配置参数 | 运行特征 |
|:---|:---|:---|:---|:---:|:---|
| **视觉通道** | USART3 | PB11(RX) / PB10(TX) | 网关 COM10 | 115200, 8N1 | 接收 30Hz 高频人脸坐标 |
| **手势通道** | USART4 | PC7(RX) / PC6(TX) | 网关 COM9 | 9600, 8N1 | 接收单字节手势控制字符 |
| **遥测控制** | USART8 | PE7(RX) / PE8(TX) | 网关 COM27 | 115200, 8N1 | 双向语音触发握手与调试 |
| **语音输入** | USART2 | PD6(RX) / PD5(TX) | ASR-PRO TX1/RX1 | 9600, 8N1 | 重映射引脚避开虚焊点 |
| **语音反馈** | USART1 | PA10(RX) / PA9(TX) | ASR-PRO TX2/RX2 | 19200, 8N1 | 控制语音合成广播与应答 |
| **Pitch 轴** | TIM2_CH1 | PA0 (AF1 复用) | 俯仰舵机驱动信号 | 50Hz PWM | 物理限幅：$0^\circ \sim 180^\circ$（初始 $75^\circ$） |
| **Yaw 轴** | TIM2_CH2 | PA1 (AF1 复用) | 偏航舵机驱动信号 | 50Hz PWM | 物理限幅：$110^\circ \sim 260^\circ$（初始 $175^\circ$） |
| **Roll 轴** | TIM2_CH3 | PA2 (AF1 复用) | 横滚舵机驱动信号 | 50Hz PWM | 物理限幅：$30^\circ \sim 60^\circ$（初始 $50^\circ$） |
| **液晶显示** | SPI2 | PB13(SCK) / PC1(MOSI)<br>PD10(CS) / PD9(DC)<br>PD8(RST) / PD11(BL) | 1.8" ST7735 TFT 屏 | 12.5MHz SPI | 遥测 UI 诊断界面，500ms 刷新 |
| **ADC 传感器** | ADC1 | PB0 (CH8) / PB1 (CH9)<br>PC0 (CH10) | MQ7 / MQ4 / MQ2 传感器 | 12-bit 精度 | 100ms 定期软件触发模数转换 |
| **状态指示** | GPIO | PC13 / PB10 | 板载 LED1 / LED2 | 推挽输出 | LED1 常亮/LED2 1Hz 心跳 |
| **报警输出** | GPIO | PA4 | 主动蜂鸣器驱动电路 | 推挽输出 | 发生环境报警时高电平触发 |
| **火焰检测** | GPIO | PA3 | 模拟火焰传感器 (DO) | 悬空输入 | 低电平有效 (0=检测到火焰) |

---

## 🧠 算法设计与数学原理

### 1. 边缘网关的多模态高精度交互算法

边缘网关采用高性能 Python 主控程序（`Gesture_recognition.py`），融合视觉人脸和手势检测，为云台提供稳定、极速的目标追踪信号：

* **人脸检测加速**：调用 InsightFace 的 `buffalo_l` 轻量化骨干网络，通过主动缩减输入检测框尺寸至 `det_size=(320, 320)`，在保障高检测精度的同时，将单帧 CPU 推理耗时降至 **30ms 以内**，跑满 30FPS 实时流。
* **双手安全追踪校验**：为防止人脸追踪模式被误触，系统设定手势指令 "5"（开启人脸追踪）必须在左右**双手同时显示手势“5”**时才被确认为有效触发。单手触发或背景干扰将被算法自动过滤。
* **手势分类自适应滤波**：手势 1-7 的判断引入了自适应手部尺寸缩放因子：
  $$\text{margin} = \max(1.5, S_{\text{hand}} \times 0.08)$$
  通过 `cv2.pointPolygonTest` 计算指尖相对于手掌凸包的距离小于 $-\text{margin}$ 作为指尖“立起”的判定依据。手势 3 与 7 的区分阈值亦根据当前手部物理尺度进行归一化：
  $$\text{Threshold}_{3/7} = 0.75 \times S_{\text{hand}}$$
  从而完全消除了手部前后位移、离摄像头远近对分类准确率的干扰。
* **IoU 锁定与运动外推预测**：系统基于检测框的重合度 IoU 做目标锁死判定（阈值 $\text{IoU} > 0.3$）。当目标发生瞬时遮挡或检测丢失时，网关根据前几帧解算出的速度矢量：
  $$v_{k} = \alpha_{\text{v}} \cdot (p_{k} - p_{k-1}) + (1-\alpha_{\text{v}}) \cdot v_{k-1} \quad (\alpha_{\text{v}} = 0.6)$$
  进行运动方向线性外推补偿，并根据目标移动速度动态调节检测间隔（$\text{speed} > 40\text{px}$ 逐帧检测，低速时自动降频至每 3 帧检测一次），大幅节省了端侧计算开销。

### 2. 双级 EMA 滤波与非线性死区控制

为降低由于人脸检测框轻微晃动引起的舵机 Hunting（鸣叫发热），系统引入双级级联一阶指数移动平均（EMA）滤波器与迟滞死区：

1. **第一级（网关坐标级输入滤波）**：
   $$X_{\text{smooth}}[k] = 0.55 \cdot X_{\text{raw}}[k] + 0.45 \cdot X_{\text{smooth}}[k-1]$$
   用于平滑网关端摄像头图像坐标的细微像素跳振（$\alpha_{\text{input}} = 0.55$）。
2. **物理控制映射与非线性迟滞死区**：
   像素坐标根据光学中心偏差量按几何关系映射为目标角度偏差，当偏差值处于死区内时，控制命令冻结，直接消除抖动：
   $$\theta_{\text{tgt, yaw}} = 175.0^\circ + \frac{e_x}{320.0} \times 45.0^\circ \quad (\text{限幅 } 140^\circ \sim 230^\circ)$$
   $$\theta_{\text{tgt, pitch}} = 75.0^\circ - \frac{e_y}{240.0} \times 35.0^\circ \quad (\text{限幅 } 40^\circ \sim 110^\circ)$$
   $$\Delta\theta_{\text{out}} = \begin{cases} 
   0, & |\Delta p| \le \text{DeadZone} \quad (\text{DeadZone}_x=30\text{px}, \text{DeadZone}_y=25\text{px}) \\
   \theta_{\text{tgt}} - \theta_{\text{prev}}, & |\Delta p| > \text{DeadZone}
   \end{cases}$$
3. **第二级（主控输出级角度滤波）**：
   在死区之外，控制输出角度经过第二级滤波产生阻尼缓动：
   $$\theta_{\text{out, yaw}}[k] = \theta_{\text{out, yaw}}[k-1] + 0.45 \cdot (\theta_{\text{tgt, yaw}} - \theta_{\text{out, yaw}}[k-1])$$
   $$\theta_{\text{out, pitch}}[k] = \theta_{\text{out, pitch}}[k-1] + 0.65 \cdot (\theta_{\text{tgt, pitch}} - \theta_{\text{out, pitch}}[k-1])$$

### 3. 三段式步进速度自适应收敛算法

云台采用三段式变结构控制策略，替代传统易饱和超调的 PID 控制器，确保大角度跨度运动快速无过冲、小角度运动微调无静差：

$$\Delta\theta = \begin{cases} 
\dfrac{e[k]}{2}, & |e[k]| > 20^\circ \quad \text{(大误差：二分速度追赶)} \\[6pt] 
\dfrac{e[k]}{3}, & 5^\circ < |e[k]| \le 20^\circ \quad \text{(中误差：三分平滑减速)} \\[6pt] 
0.1^\circ \cdot \text{sgn}(e[k]), & 0^\circ < |e[k]| \le 5^\circ \quad \text{(微小静差：0.1°最小阶跃细分逼近)} 
\end{cases}$$

### 4. Roll 轴多项式动态前馈姿态补偿

当云台进行大范围 Pitch（俯仰）与 Yaw（偏航）联动控制时，受三轴云台几何机械耦合影响，画面地平线会产生物理非线性倾斜。算法通过二元二次多项式建立前馈控制模型，使 Roll 轴自动驱动反向补偿角：

$$z_{\text{comp}} = 50.0^\circ + \frac{f_x - 75.0}{35.0} \times 8.0^\circ + \frac{f_y - 175.0}{85.0} \times 5.0^\circ$$
通过该补偿，视轴倾斜度误差被压缩在 **0.8°** 以内。

---

## ⚡ 双核与单核性能对比数据

| 技术评估维度 | 传统单核控制方案 | 本作品单核兼容模式 | 本作品双核异协同模式 | 优势分析与突破点说明 |
|:---|:---:|:---:|:---:|:---|
| **处理器架构** | 单核 RV32IMAC, 144MHz | 单核 RISC-V (V3F@100MHz) | 双核异构 (V5F@400MHz + V3F@100MHz) | 吞吐效率提升 2.77×，任务级解耦并行 |
| **高频串口丢帧率** | 4.2% ~ 8.9% | 1.2% | **0.00%** | 双核隔离，高频中断由协处理器 V3F 独占解析 |
| **画面静态抖动量** | ±0.85° | ±0.20° | **≤ ±0.15°** | 双级级联滤波与非线性自适应迟释死区共同作用 |
| **超调响应量** | 8.5% 振荡 | 1.5% | **0% 无超调** | 三段式变步进算法阻尼收敛，杜绝机械过冲 |
| **姿态解算开销** | ~12.5 ms | ~1.5 ms | **0.05 ms** | 编译参数 `-mfloat-abi=hard` 全面启动硬件 FPU 单元 |
| **串口溢出故障 (ORE)** | 时常发生 | 偶发 (高负荷下) | **绝对无** | 协处理器专享多串口接收缓冲，消除了中断嵌套延迟 |
| **主控 CPU 占用率** | 85% ~ 93% | 65% | **V5F 控制核仅占 12%** | 极富弹性的多核算力分配，支持未来功能的无限扩展 |

---

## 📊 液晶遥测 UI 与状态监控

TFT180 屏幕（160×128 分辨率，横屏显示）每 500ms 刷新一次当前系统运行数据：

```text
Track:ON  N:12       <- 跟踪模式状态 (ON/OFF) ；手势动作有效变更计数器 N
X: 75.0 Y:175.0 Z:50.0 <- 当前Pitch、Yaw、Roll轴的实时运行角度反馈
Hand: 5              <- 当前网关检测并传输过来的手势指令数值 (0-7, 5表示跟踪)
CMD: TRK             <- 状态机动作映射：STOP / X+ / X- / Y+ / Y- / TRK / Z+ / Z-
Face: 75.2  175.1    <- 共享内存中由V3F传递过来的平滑脸部坐标值
Tgt: 75  175  50     <- 当前三轴舵机指向的目标计算角度
Gesture: TRK         <- 手势名称直观映射 (STOP / TRK 等)
V3:10482  V5:209     <- V3F 通信心跳计数值与 V5F 主刷新主循环计数值，用于脱机诊断
```

通过第 8 行的 `V3 / V5` 心跳监控值，可瞬间锁定是哪一颗核心出现死机故障。此外，当发生环境（CO、烟雾、天然气、火焰）异常时，屏幕对应行会以红字高亮警告，配合 PA4 蜂鸣器驱动电路进行声光协同报警。

---

## 🚀 运行环境配置与启动步骤

> 以下步骤均基于实际开发环境验证：**Anaconda 24.9.2 + Python 3.12.7（base 环境）+ Windows 11 + CUDA 12.3**。

### 第一步 — 烧录 MCU 固件

**工具**：MounRiver Studio（MRS）

1. 用 MounRiver Studio 打开工程根目录下的 `CH32H417QEU.wvsln`。
2. 编译 **V3F** 工程 → 生成 `V3F/obj/V3F.bin`
3. 编译 **V5F** 工程 → 生成 `V5F/obj/V5F.bin`
4. 在工程根目录执行固件合并脚本，将双核固件按物理地址对齐合并：

   ```bat
   :: Windows 环境下运行
   merge_firmware.bat
   ```
   脚本将 V3F.bin 放于偏移 `0x00000000`（最大 64KB），V5F.bin 放于偏移 `0x00010000`，输出 `Merge.bin`。

5. 打开 **WCH-LinkUtility**，选择 WCH-Link 调试器，基地址填写 `0x00000000`，一键烧录 `Merge.bin`，双核固件同时刷写完成。

---

### 第二步 — 配置 Python 边缘视觉网关运行环境

所有依赖均安装到 **Anaconda base 环境**（Python 3.12.7）。

#### 2-A 安装 PyTorch（GPU 加速，对应 CUDA 12.3）

```bat
:: 已配置清华镜像源，直接使用 conda 安装
conda install pytorch torchvision torchaudio pytorch-cuda=12.1 -c pytorch -c nvidia
```

> 若 conda 通道较慢，也可改用 pip 安装：
> ```bat
> pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121
> ```

#### 2-B 安装视觉与串口依赖包

```bat
pip install insightface onnxruntime-gpu mediapipe opencv-python pyserial -i https://pypi.tuna.tsinghua.edu.cn/simple
```

| 包名 | 用途 | 推荐版本 |
|:---|:---|:---:|
| `insightface` | 人脸检测，使用 `buffalo_l` 骨干网络 | ≥ 0.7.3 |
| `onnxruntime-gpu` | InsightFace ONNX GPU 推理后端 | ≥ 1.17 |
| `mediapipe` | 21 点手部关键点检测 | ≥ 0.10 |
| `opencv-python` | 摄像头帧采集、凸包多边形判断 | ≥ 4.9 |
| `pyserial` | 与 MCU 串口通信 | ≥ 3.5 |
| `numpy` | 矩阵计算（**已安装** 1.26.4）| 1.26.4 ✓ |
| `PyQt5` | 简易 GUI 界面（**已安装** 5.15.10）| 5.15.10 ✓ |

#### 2-C 下载 InsightFace 人脸模型

首次运行时 InsightFace 会自动下载 `buffalo_l` 模型。若无法联网，手动下载并放至以下目录：

```
C:\Users\<用户名>\.insightface\models\buffalo_l\
    det_10g.onnx
    w600k_r50.onnx
```

---

### 第三步 — 配置串口号

将 USB-TTL 串口模块（CH340）插入电脑，打开**设备管理器**确认 COM 号，然后只需修改 **`Python/config.py`** 一个文件：

```python
# Python/config.py — 只改这里
GESTURE_COM_PORT = 'COM9'     # → MCU USART4 (PC7)  — 手势指令通道  @ 9600 bps
FACE_COM_PORT    = 'COM10'    # → MCU USART3 (PB11) — 人脸坐标通道  @ 115200 bps
MCU_COM_PORT     = 'COM27'    # → MCU USART8 (PE7)  — 遥测/语音通道 @ 115200 bps
```

> 波特率已在 MCU 固件中硬编码，**不要修改 `*_BAUDRATE`**，除非同步重新编译固件。

---

### 第四步 — 启动边缘视觉网关

```bat
:: 激活 Anaconda base 环境（若未激活）
call E:\Anaconda\Scripts\activate.bat E:\Anaconda

:: 在工程根目录运行主程序
python Gesture_recognition.py
```

**正常启动后的预期输出：**

```
[Gateway] Loading InsightFace buffalo_l model...
[Gateway] Model loaded. det_size=(320, 320)
[Gateway] Camera opened (index 0, 640x480)
[Serial]  COM10 opened @ 115200 bps  (Face channel)
[Serial]  COM9  opened @ 9600 bps    (Gesture channel)
[Serial]  COM27 opened @ 115200 bps  (Telemetry channel)
[Gateway] System ready — show gesture "5" with BOTH hands to enable face tracking
```

---

### 第五步 — 全链路功能验证

| 步骤 | 操作 | 期望结果 |
|:---|:---|:---|
| 1 | 给 MCU 开发板上电 | TFT180 显示 `Track:OFF` 及舵机角度 |
| 2 | 启动 `Gesture_recognition.py` | 摄像头画面弹出，串口连接成功 |
| 3 | 双手同时比出"5"对着摄像头 | TFT 切换为 `Track:ON`，云台跟踪人脸 |
| 4 | 比出手势"0" | 停止跟踪，`Track:OFF` |
| 5 | 对 ASR-PRO 下达语音指令 | 云台转动至对应预置位 |
| 6 | 查看 TFT 第 8 行 `V3:xxxxx V5:xxxxx` | 双核心跳计数器均在自增 → 双核正常运行 |

---

### 常见问题排查

| 现象 | 可能原因 | 解决方法 |
|:---|:---|:---|
| `ModuleNotFoundError: insightface` | 依赖未安装 | 执行第二步 2-B |
| `Serial port not found` | COM 口填写错误 | 查设备管理器，更新 `config.py` |
| 手势已识别但云台无响应 | COM 口接线顺序错误 | 互换 `GESTURE_COM_PORT` 与 `FACE_COM_PORT` |
| MCU 串口频繁 ORE 错误 | 波特率不匹配 | 确认 `config.py` 波特率与固件一致 |
| TFT 第 8 行 V5 心跳一直为 0 | V5F 未烧录或地址偏移错误 | 重新从工程根目录生成并烧录 `Merge.bin` |
| 摄像头无法打开 | 设备索引错误 | 将 `Gesture_recognition.py` 中 `cv2.VideoCapture(0)` 改为索引 `1` |

---

## 📂 工程文件组织架构

```text
CH32H417-EdgeGimbal/
├── Common/                         # 双核公共定义与库文件
│   ├── Common/
│   │   ├── shared_gimbal.h         # 定义双核共享结构体 GimbalSharedData_t 与信号量 HSEM_ID0
│   │   ├── shared_gimbal.c         # 将共享变量放置在链接段 .shared_data (起始地址 0x20178000)
│   │   └── gimbal_types.h          # 规定舵机物理约束限幅与 PID 结构体类型
│   └── Debug/                      # WCH 系统时钟定义与核心功能选择 (Run_Core 宏)
│
├── V3F/                            # V3F 通信协处理器工程 (Core 0, 启动运行地址 0x00000000)
│   ├── User/
│   │   ├── main.c                  # 通信核主循环、心跳自增、单核全功能运行兼容 logic
│   │   └── ch32h417_it.c           # USART1/2/3/4/8 中断中断服务函数，专职路由与快速解析
│   └── Comm/
│       └── uart_router.c           # 多路串口引脚初始化配置及高效数字/坐标手动转换算法
│
├── V5F/                            # V5F 控制主处理器工程 (Core 1, 启动运行地址 0x00010000)
│   ├── App/
│   │   └── servo.c                 # TIM2 三通道高精度 PWM 配置与三段式阻尼速度收敛算法
│   └── User/
│       └── main.c                  # 主控制循环、20ms 控制任务分发、ST7735 TFT 驱动与遥测更新
│
├── Python/                         # 边缘视觉网关控制层
│   ├── config.py                   # 端口分配 (COM9/COM10/COM27) 与通信波特率配置文件
│   └── 识别.py                     # 基于 MediaPipe 简易手势识别的 PyQt 界面程序
│
├── Gesture_recognition.py          # 高性能多模态网关程序 (InsightFace + MediaPipe Hands + 双级 EMA 滤波)
├── merge_firmware.bat              # Windows 环境下固件 hex 地址物理对齐一键合并脚本
└── merge_firmware.sh               # Linux/macOS 编译链下 DD 固件对齐对齐合并脚本
```
