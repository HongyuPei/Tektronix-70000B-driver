# Tektronix-70000B-driver

# Tektronix AWG70002A 时序控制器（`AWG70002A_Controller.py`）

基于 Python、**Broadbean** 和 **QCoDeS** 开发的双通道泰克（Tektronix）AWG70002A 任意波形发生器时序控制与编译部署脚本。专为量子控制、光学检测磁共振（ODMR）以及精密原子磁测等物理实验场景设计。

---

## 🛠️ 核心功能特点

1. **时序洗涤引擎（Fold & Merge）**
* **长空闲折叠**：自动将不含微波的长空闲切片折叠为「2400 点基础块 + 硬件重复（Repeat）」，大幅节省序列内存。
* **短片段前向拼接**：自动处理低于硬件红线（2400 点 / 160 ns @ 15 GS/s）的短切片，通过贪婪向后借调或空闲补足，绕过泰克硬件底层限制。




2. **Broadbean 原生兼容编译器**：将用户友好的时序元组列表高效转译为 Broadbean 的 `Sequence` 和 `Element` 蓝图，精准控制双通道微波正弦波频率、相位、数字幅值及 Marker 触发信号。


3. **网络全自动部署与联动**：支持通过 LAN 口一键清除远端缓存、生成本地/远端 `.seqx` 二进制固件、推送并加载序列、绑定轨道、强制切换 8-bit 高级分辨率模式，以及控制物理输出开关。


4. **常用状态快捷控制**：
* `set_awg_night()`：一键安全全关（停止播放、关闭双通道物理输出）。


* `set_awg_light()`：一键常亮光路（阻断微波，自动计算并维持满足硬件下限的无限循环激光 Marker 输出，用于实验前对光与测试荧光）。





---

## 📦 依赖环境

使用本控制器前，请确保 Python 环境中已安装以下依赖库：

* `qcodes` (QCoDeS Instrument Drivers for Tektronix AWG70002A)


* `broadbean`

* `numpy`
* `msvcrt` (Windows 终端交互中断支持)



---

## 🚀 快速上手与 API 示例

脚本内置了标准业务流示例（`__main__` 入口），可以直接参考以下方式在项目中导入和使用：

```python
import time
from AWG70K_Controller import TekAWG70kController

# 1. 实例化控制器（指定仪器 IP 与采样率）
awg_ctrl = TekAWG70kController(ip_address="169.254.92.182", sampling_rate=10e9)

# 2. 实验前准备：开启对光/测荧光模式
awg_ctrl.set_awg_light()
time.sleep(2)

# 3. 定义自定义时序切片 (flags, [freq1, phase1, freq2, phase2], duration_ns)
seq_name = "Experiment_Seq"
seq = [
    (awg_ctrl.MW1, 0.1e9, 0, 6000),  # CH1 输出 100 MHz 微波，持续 6000 ns
    (awg_ctrl.MW2, 0.2e9, 0, 6000),  # CH2 输出 200 MHz 微波，持续 6000 ns
    (awg_ctrl.LASER, 500),           # 仅开启激光 Marker，持续 500 ns
]

try:
    # 4. 编译、下发并绑定轨道 (设置功率与 12 GS/s 采样率)
    awg_ctrl.convert_seq_to_awg(
        seq, power=0.45, power2=0.4, sampling_rate=12e9, name_prefix=seq_name
    )
    
    # 5. 启动输出
    awg_ctrl.run()
    print("序列正在运行，按任意键可安全退出...")
    
    while True:
        if msvcrt.kbhit():
            msvcrt.getch()
            break
        time.sleep(0.1)

except Exception as e:
    print(f"发生异常: {e}")

finally:
    # 6. 实验结束或中断：安全返回全关暗室状态并断开连接
    awg_ctrl.set_awg_night()
    awg_ctrl.disconnect()

```

---

## 📌 时序元组（`seq_list`）格式说明

时序列表中的每个元素代表一个时间切片，支持通过按位或（`|`）组合控制标志位：

* **可用控制位（Flags）**：
* `awg_ctrl.MW1`：CH1 输出正弦微波


* `awg_ctrl.LASER` / `awg_ctrl.LIGHT`：CH1 Marker1（通常接激光 AOM）


* `awg_ctrl.COUNTER`：CH1 Marker2（通常接光子计数卡）


* `awg_ctrl.MW2`：CH2 输出正弦微波


* `awg_ctrl.CH2_M1` / `awg_ctrl.CH2_M2`：CH2 对应的 Marker 触发


* `awg_ctrl.IDLE_FLAG` / `awg_ctrl.NIGHT`：全关空闲状态
