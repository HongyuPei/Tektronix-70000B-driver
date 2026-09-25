"""Tektronix AWG70002A 时序控制器（Broadbean + QCoDeS）。

把「时序切片列表」编译成泰克 SEQX 固件并下发到仪器，主要做三件事：

1. 时序洗涤 —— 折叠长空闲、拼接短片段，绕开硬件「最小片段 2400 点」红线；
2. 编译     —— 把洗涤后的片段翻译成 Broadbean ``Sequence``；
3. 部署     —— 生成 SEQX 二进制，本地留档 + 网络推送 + 绑定轨道 + 打开输出。

用法与原理见同目录 README.md，或本文件末尾的 ``__main__`` 示例。
"""

import msvcrt
import os
import time

from broadbean import BluePrint, Element, PulseAtoms, Sequence
from qcodes.instrument_drivers.tektronix.AWG70002A import TektronixAWG70002A


class TekAWG70kController:
    """AWG70002A 双通道时序控制器。

    标志位（flags）用一个整数按位描述某个时间切片上各物理通道的状态，
    多个标志用 ``|`` 组合，例如 ``MW1 | LASER``。
    """

    # ==================== 🛠️ 1. 硬件序列控制位与特态定义 ====================
    MW1 = 1 << 1        # microwave_1 -> CH1 输出正弦波
    LASER = 1 << 2      # laser_532_aom -> CH1 Marker1
    COUNTER = 1 << 3    # counter -> CH1 Marker2
    MW2 = 1 << 4        # microwave_2 -> CH2 输出正弦波
    CH2_M1 = 1 << 5     # Ch2M1 -> CH2 Marker1
    CH2_M2 = 1 << 6     # Ch2M2 -> CH2 Marker2
    IDLE_FLAG = 0       # 全关（与 NIGHT 同值，保留两个名字以兼容原有写法）

    NIGHT = 0           # 核心定义：全关状态
    LIGHT = 1 << 2      # 核心定义：仅开启 CH1 Marker1 (LASER)，其余全关（与 LASER 同值）

    # 泰克 AWG 硬件最小片段长度（点）。短于该长度的片段无法单独播放。
    MIN_POINTS = 2400

    def __init__(self, ip_address: str = "169.254.92.182", sampling_rate: float = 15e9):
        """初始化控制器。此时并不连接仪器，连接发生在 connect() 或下发时。"""
        self.ip_address = ip_address
        self.sampling_rate = sampling_rate
        self.instrument_address = f"TCPIP0::{ip_address}::inst0::INSTR"
        self.awg = None

        # 默认微波数字幅值控制参数
        self.power = 0.5
        self.power2 = 0.5

    def connect(self):
        """连接物理仪器并初始化基础超时保护（已连接时为空操作）。"""
        if self.awg is None:
            print(f">>> 正在建立与泰克 AWG 的通信连接: {self.instrument_address} ...")
            self.awg = TektronixAWG70002A('AWG', self.instrument_address)
            self.awg.timeout(60)  # 60秒网络超时保护

    def disconnect(self):
        """断开硬件联机并释放句柄。"""
        if self.awg is not None:
            self.awg.close()
            self.awg = None
            print(">>> 已成功断开与 AWG 的连接。")

    # ==================== 🧠 2. 带有长空闲折叠与短拼接的数据流洗涤引擎 ====================
    def _preprocess_fold_and_merge_seq(self, seq_list):
        """时序洗涤引擎：绕开泰克 2400 点（160ns@15GS/s）硬红线。

        阶段 A【折叠】：不含微波的长片段，拆成「2400 点的块 + 硬件 Repeat」，
                        把几十微秒以上的空闲压缩成一次循环播放，节省序列内存。
        阶段 B【拼接】：不足 2400 点的短片段，向后贪婪借调时间凑够 2400 点。

        Returns:
            条目列表，每项含 ``flags/freq1/phase1/freq2/phase2/points/repeat/sub_segments``。
        """
        min_points = self.MIN_POINTS
        ns_to_pts = self.sampling_rate / 1e9

        # ---- 阶段 A：解析原始切片，并对无微波的长切片做硬折叠 ----
        initial_entries = []
        for step in seq_list:
            flags = step[0]
            n = len(step)

            if n == 6:
                _, freq1, phase1, freq2, phase2, duration_ns = step
            elif n == 4:
                # 4 元组按 flags 判定归属哪个通道：
                #   MW1 -> (flags, freq1, phase1, duration)
                #   MW2 -> (flags, freq2, phase2, duration)
                # 注意：flags 必须含 MW1 或 MW2，否则 duration_ns 不会被赋值。
                if flags & self.MW1:
                    _, freq1, phase1, duration_ns = step
                    freq2, phase2 = 0.0, 0.0
                elif flags & self.MW2:
                    _, freq2, phase2, duration_ns = step
                    freq1, phase1 = 0.0, 0.0
            elif n == 3:
                _, freq1, duration_ns = step
                phase1, freq2, phase2 = 0.0, 0.0, 0.0
            elif n == 2:
                _, duration_ns = step
                freq1, phase1, freq2, phase2 = 0.0, 0.0, 0.0, 0.0
            else:
                continue

            pts = int(round(duration_ns * ns_to_pts))
            if pts <= 0:
                continue

            is_mw = bool(flags & self.MW1) or bool(flags & self.MW2)

            if not is_mw and pts >= min_points * 2:
                repeat_cnt = pts // min_points
                tail_pts = pts % min_points

                # 余数太短无法单独播放，从循环里退一块出来补足尾部
                if 0 < tail_pts < min_points:
                    repeat_cnt -= 1
                    tail_pts += min_points

                if repeat_cnt > 0:
                    initial_entries.append(self._make_entry(flags, 0.0, 0.0, 0.0, 0.0, min_points, repeat_cnt))
                if tail_pts > 0:
                    initial_entries.append(self._make_entry(flags, 0.0, 0.0, 0.0, 0.0, int(tail_pts), 1))
            else:
                initial_entries.append(self._make_entry(flags, freq1, phase1, freq2, phase2, pts, 1))

        # ---- 阶段 B：对不足 2400 点的短切片做前向贪婪拼接 ----
        return self._merge_short_entries(initial_entries)

    @staticmethod
    def _make_entry(flags, freq1, phase1, freq2, phase2, points, repeat):
        """构造一个洗涤条目的字典（统一字段顺序，避免各处手写字典）。"""
        return {
            "flags": flags, "freq1": freq1, "phase1": phase1,
            "freq2": freq2, "phase2": phase2,
            "points": points, "repeat": repeat,
        }

    def _merge_short_entries(self, entries):
        """阶段 B：把不足 MIN_POINTS 的短片段向后贪婪拼接。

        注意：拼接会**原地消耗** ``entries`` 中后续片段的 ``points``，
        被完整吞并的片段不会再单独成段（这是原有语义，不是缺陷）。
        """
        min_points = self.MIN_POINTS
        cleaned_entries = []
        i = 0
        n = len(entries)

        while i < n:
            curr = entries[i].copy()

            # 硬件 Repeat 块本身已经是 2400 点的整数倍，不再参与拼接
            if curr["repeat"] > 1:
                self._ensure_sub_segments(curr)
                cleaned_entries.append(curr)
                i += 1
                continue

            while curr["points"] < min_points:
                next_idx = i + 1
                while next_idx < n and entries[next_idx]["points"] <= 0:
                    next_idx += 1

                # 后面没有可借的片段：用空闲补足
                if next_idx >= n:
                    self._pad_with_idle(curr, min_points - curr["points"])
                    break

                next_entry = entries[next_idx]
                # 硬件循环块不能被拆开借调，同样改用空闲补足
                if next_entry["repeat"] > 1:
                    self._pad_with_idle(curr, min_points - curr["points"])
                    break

                needed_pts = min_points - curr["points"]
                steal_pts = min(needed_pts, next_entry["points"])

                self._ensure_sub_segments(curr)
                curr["sub_segments"].append({
                    "flags": next_entry["flags"], "freq1": next_entry["freq1"], "phase1": next_entry["phase1"],
                    "freq2": next_entry["freq2"], "phase2": next_entry["phase2"], "points": steal_pts,
                })

                curr["points"] += steal_pts
                next_entry["points"] -= steal_pts
                if next_entry["points"] == 0:
                    i = next_idx

            self._ensure_sub_segments(curr)
            cleaned_entries.append(curr)
            i += 1

        return cleaned_entries

    @staticmethod
    def _ensure_sub_segments(entry):
        """保证条目带有 sub_segments；若还没有，就用条目自身作为第一个子片段。"""
        if "sub_segments" not in entry:
            entry["sub_segments"] = [{
                "flags": entry["flags"], "freq1": entry["freq1"], "phase1": entry["phase1"],
                "freq2": entry["freq2"], "phase2": entry["phase2"], "points": entry["points"],
            }]

    def _pad_with_idle(self, entry, needed_pts):
        """把条目补足到 MIN_POINTS，多出来的部分用空闲（IDLE_FLAG）填充。"""
        original_pts = entry["points"]
        entry["points"] = self.MIN_POINTS
        if "sub_segments" not in entry:
            entry["sub_segments"] = [{
                "flags": entry["flags"], "freq1": entry["freq1"], "phase1": entry["phase1"],
                "freq2": entry["freq2"], "phase2": entry["phase2"], "points": original_pts,
            }]
        entry["sub_segments"].append({
            "flags": self.IDLE_FLAG, "freq1": 0.0, "phase1": 0.0,
            "freq2": 0.0, "phase2": 0.0, "points": needed_pts,
        })

    # ==================== 📦 3. Broadbean 核心编译器 ====================
    def compile_seq_list_to_broadbean(self, seq_list, power=None, power2=None, name_prefix="TestSegPHY"):
        """把时序元组列表编译为原生 Broadbean ``Sequence`` 对象。

        Args:
            seq_list: 用户定义的时序元组列表（格式见 README）。
            power: CH1 正弦波数字幅值，None 表示沿用 ``self.power``。
            power2: CH2 正弦波数字幅值，None 表示沿用 ``self.power2``。
            name_prefix: 序列名，同时用作 SEQX 文件名。
        """
        p1 = self.power if power is None else power
        p2 = self.power2 if power2 is None else power2

        final_entries = self._preprocess_fold_and_merge_seq(seq_list)

        bb_seq = Sequence()
        bb_seq.name = name_prefix
        bb_seq.setSR(self.sampling_rate)
        # 硬件全局量程固定 1.0 V，功率靠片段内部的数字幅值精细控制
        bb_seq.setChannelAmplitude(1, 1.0)
        bb_seq.setChannelAmplitude(2, 1.0)

        bb_pos = 1
        for elem_idx, entry in enumerate(final_entries):
            if entry["points"] <= 0:
                continue

            bp_ch1 = self._build_blueprint(
                entry["sub_segments"], elem_idx, suffix="a",
                mw_flag=self.MW1, freq_key="freq1", phase_key="phase1",
                marker1_flag=self.LASER, marker2_flag=self.COUNTER, amp=p1,
            )
            bp_ch2 = self._build_blueprint(
                entry["sub_segments"], elem_idx, suffix="b",
                mw_flag=self.MW2, freq_key="freq2", phase_key="phase2",
                marker1_flag=self.CH2_M1, marker2_flag=self.CH2_M2, amp=p2,
            )

            elem = Element()
            elem.addBluePrint(1, bp_ch1)
            elem.addBluePrint(2, bp_ch2)
            bb_seq.addElement(bb_pos, elem)

            if entry["repeat"] > 1:
                bb_seq.setSequencingNumberOfRepetitions(bb_pos, int(entry["repeat"]))
            bb_pos += 1

        last_step = bb_pos - 1
        if last_step > 0:
            bb_seq.setSequencingGoto(last_step, 1)

        return bb_seq

    def _build_blueprint(self, sub_segments, elem_idx, suffix, mw_flag, freq_key, phase_key,
                         marker1_flag, marker2_flag, amp):
        """把一组子片段编译成单通道的 BluePrint。

        有微波的子片段插入正弦波，否则插入等长空闲；marker 按标志位整段拉高。
        ``suffix`` 只用于生成片段名（CH1 用 'a'，CH2 用 'b'）。
        """
        bp = BluePrint()
        bp.setSR(self.sampling_rate)

        for sub_idx, sub in enumerate(sub_segments):
            sub_dur_sec = sub["points"] / self.sampling_rate
            seg_name = f"subwfx{elem_idx}x{sub_idx}{suffix}"

            if sub["flags"] & mw_flag:
                bp.insertSegment(
                    sub_idx, PulseAtoms.sine,
                    (sub[freq_key], amp, 0.0, sub[phase_key]),
                    dur=sub_dur_sec, name=seg_name,
                )
            else:
                bp.insertSegment(sub_idx, PulseAtoms.waituntil, (0,), dur=sub_dur_sec, name=seg_name)

            if sub["flags"] & marker1_flag:
                bp.setSegmentMarker(seg_name, (0.0, sub_dur_sec), markerID=1)
            if sub["flags"] & marker2_flag:
                bp.setSegmentMarker(seg_name, (0.0, sub_dur_sec), markerID=2)

        return bp

    # ==================== 🚀 4. 序列编译与实体部署下发接口 ====================
    def convert_seq_to_awg(self, seq, power=1.0, power2=1.0, sampling_rate=None, name_prefix="TestSegPHY"):
        """核心部署：编译序列并通过网络推送到物理 AWG 硬件。

        只负责「下发」，不会自动开始播放；需要输出时再调用 ``run()``。
        """
        self.connect()

        if sampling_rate is not None:
            self.sampling_rate = int(sampling_rate)

        # 更新本地类幅值缓存
        self.power = power
        self.power2 = power2

        broadbean_sequence = self.compile_seq_list_to_broadbean(
            seq, power=self.power, power2=self.power2, name_prefix=name_prefix
        )

        print(">>> 正在清除 AWG 上一次遗留的波形与序列缓存...")
        self.awg.clearSequenceList()
        self.awg.clearWaveformList()

        self._push_seqx(broadbean_sequence, name_prefix, save_local=True)

        print(">>> 正在同步硬件全局采样率并映射 Sequence Track...")
        self.awg.sample_rate(self.sampling_rate)
        self._bind_tracks_and_enable(name_prefix, apply_resolution=True)

    def _push_seqx(self, bb_seq, name_prefix, save_local=False):
        """把 Broadbean 序列编译成 SEQX 二进制，本地留档（可选）并推送到仪器。

        Returns:
            远端文件名（泰克默认工作目录下的 ``<name_prefix>.seqx``）。
        """
        print(">>> 核心编译：正在通过原生接口将波形编译为物理驱动元组...")
        pkg = bb_seq.outputForSEQXFile()

        print(">>> 正在本地封装二进制固件数据流...")
        seqx_binary = self.awg.makeSEQXFile(*pkg)

        if save_local:
            local_path = os.path.abspath(f"{name_prefix}.seqx")
            with open(local_path, "wb") as f:
                f.write(seqx_binary)
            print(f"[LOCAL] 实体固件已成功保存至本地: {local_path}")

        # 远端使用纯文件名，避免泰克内部路径拼接失败导致 "cant find file"
        remote_filename = f"{name_prefix}.seqx"
        print(">>> 正在通过网口推送二进制固件至 AWG 默认工作目录...")
        self.awg.sendSEQXFile(seqx_binary, filename=remote_filename)

        print(">>> 文件传输完毕。正在通知 AWG 固件解析并载入序列...")
        self.awg.loadSEQXFile(remote_filename)
        return remote_filename

    def _bind_tracks_and_enable(self, name_prefix, apply_resolution=False):
        """把序列绑定到 CH1/CH2 轨道，并打开两个通道的物理输出。"""
        self.awg.ch1.setSequenceTrack(name_prefix, 1)
        self.awg.ch2.setSequenceTrack(name_prefix, 2)

        # 强制切换为 8 bit 高级分辨率模式（释放 Marker1/Marker2 硬件输出能力）
        if apply_resolution and hasattr(self.awg.ch1, 'resolution'):
            self.awg.ch1.resolution(8)
            self.awg.ch2.resolution(8)

        self.awg.ch1.state(1)
        self.awg.ch2.state(1)

    def run(self):
        """直接启动 AWG 输出（适用于已部署好序列的情况）。"""
        if self.awg is not None:
            self.awg.play()
        else:
            print("[ERROR] 无法启动输出：AWG 未连接。请先调用 connect() 方法建立连接。")

    def stop(self):
        """直接停止 AWG 输出。"""
        if self.awg is not None:
            self.awg.stop()
        else:
            print("[ERROR] 无法停止输出：AWG 未连接。请先调用 connect() 方法建立连接。")

    # ==================== ⚡ 5. 常态实时输出控制方法 ====================
    def set_awg_night(self) -> None:
        """NIGHT：一键全关。停止播放并切断两个通道的物理输出。"""
        if self.awg is None:
            self.connect()
        try:
            self.awg.stop()
            self.awg.ch1.state(0)
            self.awg.ch2.state(0)
        except Exception as e:
            print(f"[ERROR] 设置 NIGHT 状态失败: {e}")

    def set_awg_light(self) -> None:
        """LIGHT：一键常亮。阻断微波，仅把 CH1 Marker1（激光）持续拉高并无限循环。

        片段长度按当前采样率动态换算，保证恰好满足 2400 点硬件下限。
        """
        if self.awg is None:
            self.connect()
        try:
            self.awg.stop()

            # 时间 (秒) = 点数 / 采样率
            min_dur_sec = self.MIN_POINTS / self.sampling_rate

            static_seq = Sequence()
            static_seq.name = "LightStatic"
            static_seq.setSR(self.sampling_rate)
            static_seq.setChannelAmplitude(1, 1.0)
            static_seq.setChannelAmplitude(2, 1.0)

            elem = Element()

            bp1 = BluePrint()
            bp1.setSR(self.sampling_rate)
            bp1.insertSegment(0, PulseAtoms.waituntil, (0,), dur=min_dur_sec, name="light_on")
            bp1.setSegmentMarker("light_on", (0.0, min_dur_sec), markerID=1)  # 仅开启激光
            elem.addBluePrint(1, bp1)

            bp2 = BluePrint()
            bp2.setSR(self.sampling_rate)
            bp2.insertSegment(0, PulseAtoms.waituntil, (0,), dur=min_dur_sec, name="light_off")
            elem.addBluePrint(2, bp2)

            static_seq.addElement(1, elem)
            static_seq.setSequencingNumberOfRepetitions(1, 0)  # 0 代表无尽循环

            self._push_seqx(static_seq, "LightStatic", save_local=False)
            self._bind_tracks_and_enable("LightStatic")
            self.awg.play()
        except Exception as e:
            print(f"[ERROR] 设置 LIGHT 状态失败: {e}")


# ==================== 🚀 6. 实例主控测试业务流 ====================
if __name__ == "__main__":
    # 初始化控制器实例
    awg_ctrl = TekAWG70kController(ip_address="169.254.92.182", sampling_rate=10e9)

    # 1. 实验前准备：调用独立方法进行对光路、测荧光
    awg_ctrl.set_awg_light()
    time.sleep(2)  # 保持点亮 2 秒进行观察

    # 2. 构建实验时序
    freq1 = 0.1e9
    freq2 = 0.2e9
    seq_name = "TestSegPHY"
    seq = [
        (awg_ctrl.MW1, freq1, 0, 6000),
        (awg_ctrl.MW2, freq2, 0, 6000),
        (awg_ctrl.LASER, 500),
    ]

    print(f"\n>>> 正在解析自定义的 {len(seq)} 个时序切片...")

    try:
        # 3. 部署并直接开始正式实验
        awg_ctrl.convert_seq_to_awg(seq, power=0.45, power2=0.4, sampling_rate=12e9, name_prefix=seq_name)
        awg_ctrl.run()
        print("\n>>> 扫描序列正在全速运行中，键盘按下【任意键】将立即执行安全软着陆...")
        while True:
            if msvcrt.kbhit():
                msvcrt.getch()  # 清空缓存键
                print("\n检测到人工强制打断！正在进入安全停机流程...")
                break
            time.sleep(0.1)

    except Exception as err:
        print(f"\n[CRITICAL ERROR] 运行时捕捉到硬件异常: {err}")

    finally:
        # 4. 无论实验正常结束还是发生故障打断，最终一键切入安全全关暗室环境
        awg_ctrl.set_awg_night()
        awg_ctrl.disconnect()
