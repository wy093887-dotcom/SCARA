import re
import struct
import time

import serial
import serial.tools.list_ports
from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication

from .binary_trajectory_protocol import (
    TYPE_ACK,
    TYPE_BEGIN,
    TYPE_CHUNK,
    TYPE_NACK,
    TYPE_RUN,
    TYPE_STATUS,
    TYPE_STATUS_RSP,
    TYPE_VALIDATE,
    build_begin_payload,
    build_chunk_payload,
    build_frame,
    parse_frame,
    path_to_joint_points,
)
from .serial_protocol import build_g1_line, build_ppr_line, parse_ok_ack


class ScaraSerialMixin:
    def ui_to_mcu_xy(self, x, y):
        return x - self.L0 * 0.5, y

    def mcu_to_ui_xy(self, x, y):
        return x + self.L0 * 0.5, y

    def _write_motion_preamble(self):
        if not self.ser or not self.ser.is_open or not self.motion_preamble_needed:
            return
        for cmd in ("CLEAR_ERROR", "ENABLE 1"):
            self.ser.write((cmd + "\n").encode('ascii'))
            self.log_display.append(
                f"<font color='#bbbbbb'>TX {self.get_timestamp()} [MOTION_PREP] {cmd}</font>"
            )
        self.motion_preamble_needed = False

    def _request_controller_diagnostics(self):
        if self.ser and self.ser.is_open:
            self.ser.write(b"ERRORS\n?\n")

    def refresh_ports(self):
        self.port_combo.clear()
        ps = [p.device for p in serial.tools.list_ports.comports()]
        if ps:
            self.port_combo.addItems(ps)

    def toggle_serial(self):
        if self.ser is None or not self.ser.is_open:
            try:
                port = self.port_combo.currentText()
                if not port: return
                self.ser = serial.Serial(port, 115200, timeout=0.1)
                self.serial_status.setText("已连接")
                self.serial_status.setStyleSheet("color: green;")
                self.btn_connect.setText("断开")
                self.motion_preamble_needed = True
                self.microstep_dirty = True
                self.apply_microstep_setting()
                self.heartbeat_timer.start(200) 
            except Exception as e:
                self.log_error(f"连接失败: {e}")
        else:
            self.ser.close()
            self.heartbeat_timer.stop()
            self.serial_status.setText("未连接")
            self.serial_status.setStyleSheet("color: gray;")
            self.btn_connect.setText("连接")

    def send_heartbeat(self):
        if getattr(self, "binary_stream_active", False):
            return
        if self.ser and self.ser.is_open and not self.waiting_for_ack:
            self.heartbeat_count += 1
            self.ser.write(f"HEARTBEAT {self.heartbeat_count}\n".encode('ascii'))

    def load_motion_queue(self, path, append=False, send_path=None):
        # 统一装载轨迹，避免各个按钮重复维护计数器。
        #
        # 路径选择规则：
        # 1. send_path 不为空、控制器空闲、append=False：走二进制关节轨迹，MCU 负责 10kHz 插补。
        # 2. append=True 或控制器已有 ASCII 队列：走旧 G1 点流，用于兼容连续追加点动。
        # 3. 若当前点不在轨迹起点，_motion_path_with_current_connector 会自动补一段静默连接线。
        #
        # 调节建议：
        # - 点动/固定轨迹想更平滑，优先调 motion_mixin.py 的 BINARY_LINE_TOLERANCE_MM、PPR。
        # - 上传卡顿明显时，调 _upload_binary_motion 内 preload/chunk_size 或后台续传周期。
        if not path:
            self.log_error("未生成有效轨迹")
            return
        self.emergency_paused = False
        self.emergency_resume_path = []
        if hasattr(self, "_set_emergency_button_paused"):
            self._set_emergency_button_paused(False)
        if getattr(self, "microstep_dirty", False) and self.ser and self.ser.is_open:
            self.apply_microstep_setting()
        if not append:
            prepared = self._motion_path_with_current_connector(path, send_path)
            if prepared is None:
                return
            path, send_path = prepared
        if (
            send_path
            and not append
            and self.ser
            and self.ser.is_open
            and not self.waiting_for_ack
            and not self.point_queue
        ):
            start_x, start_y = self.cur_x, self.cur_y
            self.active_binary_send_path = list(send_path)
            self.active_preview_path = list(path)
            if self._upload_binary_motion(send_path):
                self.sent_point_id = len(send_path)
                self.total_task_points = len(send_path)
                self.task_start_time = time.time()
                self.point_queue = []
                self.last_sent_motion = None
                self.waiting_for_ack = False
                if self.plot_mode_combo.currentText() == "通讯发送内容":
                    self.history_x = [float(start_x)]
                    self.history_y = [float(start_y)]
                    self.update_plot(force=True)
                return
            self.active_binary_send_path = []
            self.log_display.append("<font color='orange'>二进制轨迹上传失败，回退到 ASCII G1 点流。</font>")
        if append and (self.waiting_for_ack or self.point_queue):
            self.point_queue.extend(path)
            self.total_task_points += len(path)
        else:
            self.sent_point_id = 0
            self.total_task_points = len(path)
            self.task_start_time = time.time()
            self.point_queue = path
        self.active_preview_path = list(path)
        self.process_queue()

    def _motion_path_with_current_connector(self, path, send_path=None):
        if not path:
            return None
        first = path[0]
        start_x, start_y = float(self.cur_x), float(self.cur_y)
        first_x, first_y = float(first[0]), float(first[1])
        if ((first_x - start_x) * (first_x - start_x) + (first_y - start_y) * (first_y - start_y)) <= 0.0025:
            return path, send_path
        try:
            feed_mm_s = max(0.1, float(first[2]) / 60.0) if len(first) > 2 else 1.0
            connector = self.generate_linear_path(start_x, start_y, first_x, first_y, feed_mm_s, silent=True)
            if not connector:
                self.log_error(
                    f"无法规划到轨迹起点的连接段: 当前({start_x:.2f},{start_y:.2f}) -> 起点({first_x:.2f},{first_y:.2f})"
                )
                return None
            path = connector + list(path)
            if send_path:
                if hasattr(self, "generate_binary_send_from_path"):
                    connector_send = self.generate_binary_send_from_path(connector, feed_mm_s, start=(start_x, start_y))
                else:
                    connector_send = connector
                send_path = list(connector_send) + list(send_path)
            if hasattr(self, "set_planned_preview"):
                self.set_planned_preview(path, getattr(self, "preview_label", "轨迹预览") or "轨迹预览")
            self.log_display.append(
                f"<font color='#bbbbbb'>自动补充到轨迹起点连接段: ({start_x:.1f},{start_y:.1f}) -> ({first_x:.1f},{first_y:.1f})</font>"
            )
            return path, send_path
        except Exception as exc:
            self.log_error(f"轨迹起点连接段规划失败: {exc}")
            return None

    def _read_binary_frame(self, timeout_s=1.0):
        if not self.ser or not self.ser.is_open:
            return None
        deadline = time.time() + timeout_s
        buf = bytearray()
        expected_len = None
        while time.time() < deadline:
            chunk = self.ser.read(1)
            if not chunk:
                self._pump_ui_events()
                continue
            byte = chunk[0]
            if len(buf) == 0:
                if byte == 0xA5:
                    buf.append(byte)
                continue
            if len(buf) == 1:
                if byte == 0x5A:
                    buf.append(byte)
                elif byte == 0xA5:
                    buf[:] = b"\xA5"
                else:
                    buf.clear()
                continue
            buf.append(byte)
            if len(buf) == 8 and expected_len is None:
                payload_len = struct.unpack_from("<H", buf, 6)[0]
                expected_len = 10 + payload_len
                if expected_len > 512:
                    buf.clear()
                    expected_len = None
                    continue
            if expected_len is not None and len(buf) >= expected_len:
                return parse_frame(bytes(buf[:expected_len]))
        return None

    def _pump_ui_events(self):
        app = QApplication.instance()
        if app is not None:
            app.processEvents()

    def _send_binary_frame_expect_ack(self, frame_type, seq, payload=b"", timeout_s=1.0):
        self.ser.write(build_frame(frame_type, seq, payload))
        frame = self._read_binary_frame(timeout_s=timeout_s)
        if frame is None:
            raise TimeoutError(f"binary frame 0x{frame_type:02X} timeout")
        if frame.seq != (seq & 0xFFFF):
            raise ValueError(f"binary seq mismatch: tx={seq & 0xFFFF}, rx={frame.seq}")
        if frame.frame_type == TYPE_NACK:
            err = frame.payload[1] if len(frame.payload) > 1 else 255
            raise ValueError(f"binary NACK type=0x{frame_type:02X} err={err}")
        if frame.frame_type != TYPE_ACK:
            raise ValueError(f"unexpected binary response type=0x{frame.frame_type:02X}")
        if len(frame.payload) >= 2 and frame.payload[0] == frame_type and frame.payload[1] != 0:
            raise ValueError(f"binary ACK err={frame.payload[1]}")
        return frame

    def _send_binary_status(self, seq, timeout_s=1.0):
        self.ser.write(build_frame(TYPE_STATUS, seq, b""))
        frame = self._read_binary_frame(timeout_s=timeout_s)
        if frame is None:
            raise TimeoutError("binary status timeout")
        if frame.seq != (seq & 0xFFFF):
            raise ValueError(f"binary status seq mismatch: tx={seq & 0xFFFF}, rx={frame.seq}")
        if frame.frame_type != TYPE_STATUS_RSP:
            raise ValueError(f"unexpected status response type=0x{frame.frame_type:02X}")
        payload = frame.payload
        if len(payload) < 19:
            raise ValueError("short binary status payload")
        queued = struct.unpack_from("<H", payload, 2)[0]
        free = struct.unpack_from("<H", payload, 4)[0]
        accepted = struct.unpack_from("<I", payload, 6)[0]
        executed = struct.unpack_from("<I", payload, 10)[0]
        total = struct.unpack_from("<I", payload, 14)[0]
        state = payload[18]
        return {
            "queued": queued,
            "free": free,
            "accepted": accepted,
            "executed": executed,
            "total": total,
            "state": state,
        }

    def _send_ascii_wait_ok(self, line, timeout_s=1.5):
        deadline = time.time() + timeout_s
        self.ser.write((line + "\n").encode("ascii"))
        self.log_display.append(f"<font color='#bbbbbb'>TX {self.get_timestamp()} [BINARY_PREP] {line}</font>")
        while time.time() < deadline:
            raw = self.ser.readline().decode("ascii", errors="ignore").strip()
            if not raw:
                continue
            if raw.lower().startswith("ok"):
                self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} [BINARY_PREP] {raw}</font>")
                return True
            if raw.startswith("<"):
                continue
            if "error:" in raw.lower():
                raise ValueError(raw)
        raise TimeoutError(f"ASCII prep timeout: {line}")

    def _upload_binary_motion(self, send_path):
        """上传二进制关节轨迹并启动下位机插补。

        这里先预装 preload 个关键点，然后立即 RUN，剩余关键点交给 QTimer 后台续传。
        这样复杂轨迹不会长时间占住 UI 线程，界面也不会在点击后“没反应”。
        """
        if not send_path:
            return False
        start_xy = (self.cur_x, self.cur_y)
        try:
            points = path_to_joint_points(send_path, self.kinematics, self.current_ppr, start_xy=start_xy)
            if not points:
                self.log_error("二进制轨迹无有效关节目标点")
                return False
        except Exception as exc:
            self.log_error(f"二进制轨迹转换失败: {exc}")
            return False

        read_was_active = self.read_timer.isActive()
        heartbeat_was_active = self.heartbeat_timer.isActive()
        if read_was_active:
            self.read_timer.stop()
        if heartbeat_was_active:
            self.heartbeat_timer.stop()
        self.timeout_timer.stop()

        try:
            for cmd in ("STOP", "CLEAR_ERROR", "ENABLE 1"):
                self._send_ascii_wait_ok(cmd)
            self.motion_preamble_needed = False
            seq = getattr(self, "binary_seq", 1) & 0xFFFF
            self._send_binary_frame_expect_ack(TYPE_BEGIN, seq, build_begin_payload(len(points)))
            seq = (seq + 1) & 0xFFFF
            # chunk_size：每个二进制 CHUNK 包含的关键点数量。
            # 调大可减少协议开销，但单次串口阻塞时间更长；调小 UI 更灵敏但包数更多。
            chunk_size = 20
            sent = 0
            # preload：启动前预装点数。太小容易下位机欠载，太大点击后等待变长。
            preload = min(len(points), 100)
            while sent < preload:
                take = min(chunk_size, preload - sent)
                chunk = points[sent : sent + take]
                self._send_binary_frame_expect_ack(TYPE_CHUNK, seq, build_chunk_payload(chunk))
                seq = (seq + 1) & 0xFFFF
                sent += take
                self._pump_ui_events()
            self._send_binary_frame_expect_ack(TYPE_VALIDATE, seq)
            seq = (seq + 1) & 0xFFFF
            self._send_binary_frame_expect_ack(TYPE_RUN, seq)
            seq = (seq + 1) & 0xFFFF
            self._start_binary_stream(points, sent, seq, chunk_size)
            self.log_display.append(
                f"<font color='#00ff99'>二进制关节轨迹已启动: keypoints={len(points)} preload={sent} control_hz=10000</font>"
            )
            return True
        except Exception as exc:
            self.log_error(f"二进制轨迹上传错误: {exc}")
            return False
        finally:
            if read_was_active:
                self.read_timer.start(10)
            if heartbeat_was_active:
                self.heartbeat_timer.start(200)

    def _start_binary_stream(self, points, sent, seq, chunk_size):
        """启动后台续传，把 RUN 之后还没下发的关键点分批补给 MCU。"""
        self.binary_stream_points = list(points)
        self.binary_stream_sent = int(sent)
        self.binary_stream_seq = int(seq) & 0xFFFF
        self.binary_stream_chunk_size = int(chunk_size)
        self.binary_stream_active = self.binary_stream_sent < len(self.binary_stream_points)
        if not self.binary_stream_active:
            self.binary_seq = self.binary_stream_seq
            return
        if not hasattr(self, "binary_stream_timer"):
            self.binary_stream_timer = QTimer()
            self.binary_stream_timer.timeout.connect(self._continue_binary_stream)
        # 续传周期 20ms：兼顾 UI 响应和串口缓冲余量；若状态中出现 underrun，可适当调小。
        self.binary_stream_timer.start(20)

    def _stop_binary_stream(self):
        timer = getattr(self, "binary_stream_timer", None)
        if timer is not None and timer.isActive():
            timer.stop()
        self.binary_stream_active = False
        self.binary_stream_points = []
        self.binary_stream_sent = 0

    def _continue_binary_stream(self):
        """后台续传定时器回调：查询 MCU 空余缓冲，再补发少量 CHUNK。"""
        if not getattr(self, "binary_stream_active", False):
            return
        if not self.ser or not self.ser.is_open:
            self._stop_binary_stream()
            return
        points = getattr(self, "binary_stream_points", [])
        sent = int(getattr(self, "binary_stream_sent", 0))
        seq = int(getattr(self, "binary_stream_seq", getattr(self, "binary_seq", 1))) & 0xFFFF
        if sent >= len(points):
            self.binary_seq = seq
            self._stop_binary_stream()
            self.log_display.append("<font color='#00ff99'>二进制轨迹剩余关键点已全部下发。</font>")
            return

        read_was_active = self.read_timer.isActive()
        heartbeat_was_active = self.heartbeat_timer.isActive()
        if read_was_active:
            self.read_timer.stop()
        if heartbeat_was_active:
            self.heartbeat_timer.stop()
        try:
            status = self._send_binary_status(seq, timeout_s=0.15)
            seq = (seq + 1) & 0xFFFF
            free = int(status["free"])
            chunks_this_tick = 0
            # 每个 UI tick 最多补两包，避免串口读写长时间占用主线程。
            while free > 0 and sent < len(points) and chunks_this_tick < 2:
                take = min(int(getattr(self, "binary_stream_chunk_size", 20)), free, len(points) - sent)
                if take <= 0:
                    break
                self._send_binary_frame_expect_ack(
                    TYPE_CHUNK,
                    seq,
                    build_chunk_payload(points[sent : sent + take]),
                    timeout_s=0.15,
                )
                seq = (seq + 1) & 0xFFFF
                sent += take
                free -= take
                chunks_this_tick += 1
            self.binary_stream_sent = sent
            self.binary_stream_seq = seq
            if sent >= len(points):
                self.binary_seq = seq
                self._stop_binary_stream()
                self.log_display.append("<font color='#00ff99'>二进制轨迹剩余关键点已全部下发。</font>")
        except Exception as exc:
            self.log_error(f"二进制轨迹后台续传错误: {exc}")
            self._stop_binary_stream()
        finally:
            if read_was_active:
                self.read_timer.start(10)
            if heartbeat_was_active and not getattr(self, "binary_stream_active", False):
                self.heartbeat_timer.start(200)
            self._pump_ui_events()

    def on_microstep_changed(self):
        self.microstep_dirty = True
        if self.ser and self.ser.is_open and not self.waiting_for_ack and not self.point_queue:
            self.apply_microstep_setting()

    def apply_microstep_setting(self):
        try:
            ppr = int(self.microstep_combo.currentText())
        except Exception as exc:
            self.log_error(f"细分/脉冲参数错误: {exc}")
            return False
        if self.waiting_for_ack or self.point_queue:
            self.log_display.append("<font color='yellow'>PPR 参数将在当前队列结束后应用</font>")
            return False
        if not (self.ser and self.ser.is_open):
            self.microstep_dirty = True
            return False
        line = build_ppr_line(ppr)
        if self.send_ascii_line(line, "PPR"):
            self.current_ppr = ppr
            self.microstep_dirty = False
            self.send_ascii_line("PARAMS", "PARAMS")
            return True
        return False

    def check_serial_feedback(self):
        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
            try:
                raw = self.ser.readline().decode('ascii', errors='ignore').strip()
                if not raw: return

                # 1. 独立处理下位机健康监控反馈 (心跳 OK)
                if raw.startswith("OK HEARTBEAT"):
                    tick_match = re.search(r'tick=(\d+)', raw)
                    err_match = re.search(r'err=(\d+)', raw)
                    gbuf_match = re.search(r'gbuf=\d+,(\d+)', raw)
                    if tick_match: self.lbl_mcu_tick.setText(f"MCU时间: {tick_match.group(1)} ms")
                    if err_match: self.lbl_mcu_err.setText(f"错误码: {err_match.group(1)}")
                    if gbuf_match: self.lbl_mcu_gbuf.setText(f"缓冲区占用: {gbuf_match.group(1)} / 32")
                    # 心跳响应在此处终结，绝对不执行后续运动队列逻辑
                    return 

                # 2. 处理运动指令 ACK；系统 OK 只记录，不推进运动队列。
                if raw.lower().startswith("ok"):
                    ts = self.get_timestamp()
                    ack = parse_ok_ack(raw, self.last_sent_package.strip())

                    if not (ack.rx_checksum and ack.rx_line):
                        self.log_display.append(f"<font color='#98c379'>RX {ts} [OK] {raw}</font>")
                        return

                    if not ack.matched:
                        self.log_display.append(
                            f"<font color='orange'>RX {ts} [OUT_OF_BAND_ACK] {raw}</font>"
                        )
                        self.log_display.append(
                            f"<font color='orange'>MISMATCH expected_cs={ack.expected_checksum} rx_cs={ack.rx_checksum} expected_line={self.last_sent_package.strip()}</font>"
                        )
                        return

                    if not self.waiting_for_ack:
                        self.log_display.append(f"<font color='orange'>RX {ts} [STALE_ACK] {raw}</font>")
                        return

                    self.timeout_timer.stop()
                    self.waiting_for_ack = False
                    self.ack_timeout_count = 0
                    self.stream_waiting_buffer = False
                    self.last_sent_motion = None

                    self.log_display.append(f"<font color='#ffffff'>RX {ts} [ACK {self.sent_point_id}/{self.total_task_points}] {raw}</font>")
                    self.log_display.append(f"<font color='#00ff99'>MATCH cs={ack.expected_checksum} line=OK</font>")
                    
                    # ACK line 是下位机回显的目标命令，不是真实运动反馈；真实反馈只使用状态帧 M:x,y。
                    # 只有匹配当前 G-code 的 ACK 才能推进队列，避免 OK ENABLE/OK ZERO 误触发点动发送。
                    self.process_queue()
                    return

                # 3. 处理主动推送的状态包 <...>
                if raw.startswith('<') and '>' in raw:
                    bf_match = re.search(r'Bf:(\d+),(\d+)', raw)
                    if bf_match:
                        self.mcu_planner_free = int(bf_match.group(2))
                        self.lbl_mcu_gbuf.setText(f"Planner free: {self.mcu_planner_free} / 32")
                        if self.stream_waiting_buffer and self.mcu_planner_free > 0 and not self.waiting_for_ack:
                            self.stream_waiting_buffer = False
                            QTimer.singleShot(50, self.process_queue)

                    q_match = re.search(r'Q:(\d+)', raw)
                    if q_match: self.lbl_mcu_queue.setText(f"队列负载(Q): {q_match.group(1)}")

                    jt_match = re.search(r'JT:([^,|]+),(\d+),(\d+),(\d+),(\d+)', raw)
                    if jt_match and hasattr(self, "lbl_mcu_interp"):
                        jt_state = jt_match.group(1)
                        accepted = int(jt_match.group(2))
                        executed = int(jt_match.group(3))
                        queued = int(jt_match.group(4))
                        free = int(jt_match.group(5))
                        interp_text = f"插补: {jt_state}  已执行 {executed}/{accepted}\n队列 {queued}  空余 {free}"
                        ju_match = re.search(r'JU:(\d+),(\d+),(\d+)', raw)
                        if ju_match:
                            interp_text += (
                                f"  欠载 {ju_match.group(1)}t"
                                f"  间隔 {ju_match.group(2)}t"
                                f"\n低水 {ju_match.group(3)}"
                            )
                        self.lbl_mcu_interp.setText(interp_text)

                    hz_match = re.search(r'Hz:(\d+)', raw)
                    if hz_match and hasattr(self, "lbl_mcu_hz"):
                        self.lbl_mcu_hz.setText(f"控制频率: {hz_match.group(1)} Hz")

                    pulse_match = re.search(r'P:([-?\d]+),([-?\d]+)', raw)
                    a1_match = re.search(r'A1:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    a2_match = re.search(r'A2:(\d+),(\d+),([-?\d]+),([-?\d]+)', raw)
                    if pulse_match:
                        self.feedback_p1 = int(pulse_match.group(1))
                        self.feedback_p2 = int(pulse_match.group(2))
                    if a1_match:
                        self.feedback_a1_pps = (int(a1_match.group(3)), int(a1_match.group(4)))
                    if a2_match:
                        self.feedback_a2_pps = (int(a2_match.group(3)), int(a2_match.group(4)))
                    
                    h_match = re.search(r'H:(\d),(\d)', raw)
                    if h_match:
                        h1, h2 = int(h_match.group(1)), int(h_match.group(2))
                        if not self.board_only_debug:
                            self.home_sensor_triggered = (h1 == 1 or h2 == 1)
                        
                    hs_match = re.search(r'HS:(\w+)', raw)
                    if hs_match and hs_match.group(1).lower() == "done": 
                        self.is_homed = True
                    
                    match = re.search(r'M:([-?\d.]+),([-?\d.]+)', raw)
                    if match:
                        rx, ry = self.mcu_to_ui_xy(float(match.group(1)), float(match.group(2)))
                        q1, q2 = self.inverse_kinematics(rx, ry)
                        if hasattr(self, "feedback_pose_label"):
                            self.feedback_pose_label.setText(f"回传末端: X={rx:.3f}, Y={ry:.3f}")
                        if hasattr(self, "feedback_joint_label"):
                            if q1 is None or q2 is None:
                                self.feedback_joint_label.setText("回传角度: M1=不可解, M2=不可解")
                            else:
                                self.feedback_joint_label.setText(f"回传角度: M1={q1:.2f} deg, M2={q2:.2f} deg")
                        if hasattr(self, "feedback_pulse_label"):
                            p1 = getattr(self, "feedback_p1", None)
                            p2 = getattr(self, "feedback_p2", None)
                            a1 = getattr(self, "feedback_a1_pps", (None, None))
                            a2 = getattr(self, "feedback_a2_pps", (None, None))
                            self.feedback_pulse_label.setText(
                                f"脉冲/PPS: P={p1 if p1 is not None else '--'},{p2 if p2 is not None else '--'}  "
                                f"A1={a1[0] if a1[0] is not None else '--'}/{a1[1] if a1[1] is not None else '--'}  "
                                f"A2={a2[0] if a2[0] is not None else '--'}/{a2[1] if a2[1] is not None else '--'}"
                            )
                        if hasattr(self, "append_feedback_point"):
                            self.append_feedback_point(rx, ry)
                        if getattr(self, "velocity_monitor", None) is not None:
                            self.velocity_monitor.process_new_data(f"X{rx:.3f} Y{ry:.3f}")
                        if self.plot_mode_combo.currentText() == "通讯接收内容":
                            self.cur_x, self.cur_y = rx, ry
                            ik = self.inverse_kinematics(rx, ry)
                            if ik and ik[0] is not None:
                                self.update_plot(ik[0], ik[1])
                        else:
                            self.update_plot()
                    return

                # 4. 处理错误报警
                if "error:" in raw.lower():
                    if raw.lower().startswith("error:8"):
                        self.log_display.append(
                            "<font color='orange'>RX error:8，下位机 pending/buffer 忙；暂停发送、查询状态，不急停。</font>"
                        )
                        self.stream_waiting_buffer = True
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        if self.last_sent_motion is not None:
                            self.point_queue.insert(0, self.last_sent_motion)
                            self.last_sent_motion = None
                            self.sent_point_id = max(0, self.sent_point_id - 1)
                        if self.ser and self.ser.is_open:
                            self.ser.write(b"?\n")
                        return
                    self.log_display.append(f"<font color='red'>控制器报警: {raw}</font>")
                    if self.board_only_debug and raw.lower().startswith("error:5"):
                        # 仅连接控制板调试时，error:5 常见原因是下位机已在回零/忙状态。
                        # 此时不自动急停，避免反复把控制器打入 ESTOP；接入真实 HOME 开关后关闭 board_only_debug。
                        self.is_homed = True
                        self.home_sensor_triggered = False
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        return
                    if raw.lower().startswith("error:15"):
                        self.log_display.append(
                            "<font color='orange'>运动被下位机拒绝，暂停队列并查询 ERRORS/状态；不自动急停。</font>"
                        )
                        self.point_queue = []
                        self.waiting_for_ack = False
                        self.timeout_timer.stop()
                        self.last_sent_motion = None
                        self.motion_preamble_needed = True
                        self._request_controller_diagnostics()
                        return
                    self.emergency_stop()
                    return

                # 5. 其他杂项信息
                self.log_display.append(f"<font color='#98c379'>RX {self.get_timestamp()} {raw}</font>")
            except Exception as e: 
                pass

    def handle_timeout(self):
        if self.waiting_for_ack and self.ser and self.ser.is_open:
            self.ack_timeout_count += 1
            self.log_display.append(
                f"<font color='orange'>等待 ok 超时 {self.ack_timeout_count} 次，查询状态，不重发 G-code</font>"
            )
            self.stream_waiting_buffer = True
            self.ser.write(b"?\n")
            self.timeout_timer.start(1500) 

    def process_queue(self):
        if getattr(self, "emergency_paused", False):
            return
        if not self.point_queue or self.waiting_for_ack: return
        if (not self.board_only_debug) and self.home_sensor_triggered:
            self.log_error("传感器触发，轨迹终止")
            self.point_queue = []
            return
        if (not self.board_only_debug) and (not self.is_homed) and "$H" not in self.last_sent_package:
             self.log_error("未回零，请先执行寻原点")
             self.point_queue = []
             return
        tx, ty, feed_rate, slt = self.point_queue.pop(0)
        self.last_sent_motion = (tx, ty, feed_rate, slt)
        self.sent_point_id += 1
        mcu_tx, mcu_ty = self.ui_to_mcu_xy(tx, ty)
        gcode_raw = build_g1_line(mcu_tx, mcu_ty, feed_rate, self.sent_point_id, limit_checked=True)
        gcode_line = gcode_raw + "\n"
        self.last_sent_cs = self.calculate_checksum(gcode_raw)
        self.last_sent_package = gcode_raw
        if self.plot_mode_combo.currentText() == "通讯发送内容":
            self.cur_x, self.cur_y = tx, ty
            if slt: self.history_x, self.history_y = [tx], [ty]
            else: self.history_x.append(tx); self.history_y.append(ty)
            ik = self.inverse_kinematics(tx, ty)
            if ik and ik[0] is not None: self.update_plot(ik[0], ik[1])
        ts = self.get_timestamp()
        log_msg = f"TX {ts} [point {self.sent_point_id}/{self.total_task_points}] cs={self.last_sent_cs} len={len(gcode_line)} line={gcode_raw}"
        self.log_display.append(f"<font color='#ffffff'>{log_msg}</font>")
        if self.ser and self.ser.is_open:
            self._write_motion_preamble()
            self.ser.write(gcode_line.encode('ascii'))
            self.waiting_for_ack = True  
            self.timeout_timer.start(1000) 
        else:
            # ---- ??????????? / ???????? dt ----
            prev_x, prev_y = self.cur_x, self.cur_y
            self.cur_x, self.cur_y = tx, ty
            if slt:
                self.history_x, self.history_y = [tx], [ty]
            else:
                self.history_x.append(tx)
                self.history_y.append(ty)

            import math as _math
            dx = tx - prev_x
            dy = ty - prev_y
            dist = _math.hypot(dx, dy)
            feed_mm_s = feed_rate / 60.0
            dt = dist / feed_mm_s if feed_mm_s > 0.001 else self.dt

            monitor = getattr(self, "velocity_monitor", None)
            if monitor is not None and hasattr(monitor, "process_tcp_point"):
                monitor.process_tcp_point(tx, ty, dt)

            ik = self.inverse_kinematics(tx, ty)
            if ik and ik[0] is not None:
                self.update_plot(ik[0], ik[1])
            QTimer.singleShot(10, self.process_queue)
