import math
import numpy as np


class ScaraMotionMixin:
    JOINT_LIMITS_DEG = ((-180.0, 180.0), (0.0, 360.0))
    ARC_SEGMENT_MM = 2.0

    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)

    def check_workspace_safety(self, x, y):
        return self.kinematics.is_reachable(x, y, margin=5.0)

    def _read_float(self, widget, name, positive=False):
        text = widget.text().strip()
        if not text:
            raise ValueError(f"{name}不能为空")
        try:
            value = float(text)
        except ValueError as exc:
            raise ValueError(f"{name}必须是数字，当前为 {text!r}") from exc
        if positive and value <= 0.0:
            raise ValueError(f"{name}必须大于 0，当前为 {value:g}")
        return value

    def _limit_violations_at(self, x, y, index):
        c = self.kinematics.config
        violations = []
        d1 = math.hypot(x, y)
        d2 = math.hypot(x - c.base_distance, y)

        if y < c.min_y:
            violations.append(f"点{index}: Y 低于下限 {c.min_y:.1f}mm，超出 {c.min_y - y:.2f}mm")
        if d1 < c.min_anchor_dist:
            violations.append(f"点{index}: 左基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {c.min_anchor_dist - d1:.2f}mm")
        if d1 > c.max_anchor_dist:
            violations.append(f"点{index}: 左基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {d1 - c.max_anchor_dist:.2f}mm")
        if d2 < c.min_anchor_dist:
            violations.append(f"点{index}: 右基座距离低于下限 {c.min_anchor_dist:.1f}mm，超出 {c.min_anchor_dist - d2:.2f}mm")
        if d2 > c.max_anchor_dist:
            violations.append(f"点{index}: 右基座距离超过上限 {c.max_anchor_dist:.1f}mm，超出 {d2 - c.max_anchor_dist:.2f}mm")

        q1, q2 = self.inverse_kinematics(x, y)
        if q1 is None or q2 is None:
            violations.append(f"点{index}: 五连杆无逆解 X={x:.2f}, Y={y:.2f}")
            return violations

        for axis, angle in ((1, q1), (2, q2)):
            low, high = self.JOINT_LIMITS_DEG[axis - 1]
            if angle < low:
                violations.append(f"点{index}: M{axis} 低于下限 {low:.1f}deg，超出 {low - angle:.2f}deg")
            if angle > high:
                violations.append(f"点{index}: M{axis} 超过上限 {high:.1f}deg，超出 {angle - high:.2f}deg")

        q1_rad = math.radians(q1)
        q2_rad = math.radians(q2)
        left_elbow = (
            c.active_link * math.cos(q1_rad),
            c.active_link * math.sin(q1_rad),
        )
        right_elbow = (
            c.base_distance + c.active_link * math.cos(q2_rad),
            c.active_link * math.sin(q2_rad),
        )
        if left_elbow[0] >= right_elbow[0]:
            violations.append(f"点{index}: 左右主动臂交叉，交叉量 {left_elbow[0] - right_elbow[0]:.2f}mm")
        if left_elbow[1] < 0.0:
            violations.append(f"点{index}: 左主动臂低于基座线，超出 {-left_elbow[1]:.2f}mm")
        if right_elbow[1] < 0.0:
            violations.append(f"点{index}: 右主动臂低于基座线，超出 {-right_elbow[1]:.2f}mm")

        return violations

    def validate_trajectory_points(self, points, label="轨迹"):
        violations = []
        total = 0
        for index, point in enumerate(points, start=1):
            x, y = float(point[0]), float(point[1])
            point_violations = self._limit_violations_at(x, y, index)
            total += len(point_violations)
            if point_violations and len(violations) < 5:
                violations.extend(point_violations[: 5 - len(violations)])

        if total:
            self.log_error(f"{label}预检查失败，共 {total} 项超限，已拦截发送")
            for item in violations:
                self.log_error(item)
            if total > len(violations):
                self.log_error(f"其余 {total - len(violations)} 项超限已省略")
            return False
        return True

    def generate_linear_path(self, x1, y1, x2, y2, speed_max, v_start=0.0, v_end=0.0, silent=False):
        planned = self.path_planner.plan_line(
            (x1, y1),
            (x2, y2),
            feed_mm_s=speed_max,
            start_speed=v_start,
            end_speed=v_end,
            silent=silent,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, "直线路径"):
            return []
        return path

    def generate_arc_path(self, x1, y1, x2, y2, radius, clockwise, speed_max, silent=False):
        planned = self.path_planner.plan_arc(
            (x1, y1),
            (x2, y2),
            radius,
            clockwise=clockwise,
            feed_mm_s=speed_max,
            start_speed=0.0,
            end_speed=0.0,
            silent=silent,
        )
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, "圆弧路径"):
            return []
        return path

    def generate_polyline_path(self, points, speed_max, silent_first=False):
        # 上位机按真实弧长预先规划速度；下位机只接收带 F 的 G1 点流。
        clean_points = []
        for x, y in points:
            if not clean_points or np.hypot(x - clean_points[-1][0], y - clean_points[-1][1]) > 0.001:
                clean_points.append((x, y))
        if len(clean_points) < 2:
            self.log_error("轨迹点过少，至少需要起点和终点")
            return []
        if not self.validate_trajectory_points(clean_points, "轨迹控制点"):
            return []
        planned = self.path_planner.plan_polyline(clean_points, feed_mm_s=speed_max, silent_first=silent_first)
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, "轨迹采样点"):
            return []
        return path

    def generate_geometry_path(self, segments, speed_max, silent_first=False, label="固定轨迹"):
        control_points = []
        for segment in segments:
            if not control_points:
                control_points.append(segment.start)
            control_points.append(segment.end)
        if len(control_points) < 2:
            self.log_error(f"{label}点过少，至少需要起点和终点")
            return []
        if not self.validate_trajectory_points(control_points, f"{label}控制点"):
            return []
        planned = self.path_planner.plan_segments(segments, feed_mm_s=speed_max, silent_first=silent_first)
        path = [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]
        if path and not self.validate_trajectory_points(path, f"{label}采样点"):
            return []
        return path

    def generate_arc_control_points(self, start, end, radius, clockwise):
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        chord = math.hypot(dx, dy)
        if chord < 0.001:
            raise ValueError("圆弧起点和终点重合；当前界面未定义整圆圆心")
        min_radius = chord * 0.5
        if radius < min_radius:
            raise ValueError(f"圆弧半径过小，至少需要 {min_radius:.2f}mm，当前为 {radius:.2f}mm")

        mx = (sx + ex) * 0.5
        my = (sy + ey) * 0.5
        h = math.sqrt(max(0.0, radius * radius - min_radius * min_radius))
        nx = -dy / chord
        ny = dx / chord
        centers = ((mx + h * nx, my + h * ny), (mx - h * nx, my - h * ny))

        candidates = []
        for cx, cy in centers:
            a0 = math.atan2(sy - cy, sx - cx)
            a1 = math.atan2(ey - cy, ex - cx)
            if clockwise:
                delta = -((a0 - a1) % (2.0 * math.pi))
            else:
                delta = (a1 - a0) % (2.0 * math.pi)
            if abs(delta) < 1e-9:
                delta = -2.0 * math.pi if clockwise else 2.0 * math.pi
            candidates.append((abs(delta), cx, cy, a0, delta))

        _, cx, cy, a0, delta = min(candidates, key=lambda item: item[0])
        arc_len = abs(delta) * radius
        segments = max(8, int(math.ceil(arc_len / self.ARC_SEGMENT_MM)))
        segments = min(segments, 5000)
        points = []
        for i in range(segments + 1):
            t = i / segments
            a = a0 + delta * t
            points.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
        points[0] = start
        points[-1] = end
        return points

    def start_recording(self):
        self.teach_data = [] 
        self.is_recording = True 
        self.log_display.append("<font color='green'>录制开始</font>")
        
    def stop_recording(self): 
        self.is_recording = False 
        self.log_display.append(f"录制结束")
        
    def record_single_point(self):
        if self.inverse_kinematics(self.cur_x, self.cur_y)[0] is not None:
            self.teach_points.append((self.cur_x,self.cur_y))
            self.log_display.append(f"<font color='cyan'>手动记录点: ({self.cur_x:.1f}, {self.cur_y:.1f})</font>")
                
    def clear_teach_points(self): 
        self.teach_points = []
        self.teach_data = []
        self.log_display.append("轨迹已清空")
    
    def start_playback(self):
        v = float(self.hw_speed_input.text())
        if self.teach_points:
            pts = [(self.cur_x, self.cur_y)] + self.teach_points
            path = self.generate_polyline_path(pts, v, silent_first=True)
            self.preview_planned_path(path, "示教复现")
            self.load_motion_queue(path)

    def preview_planned_path(self, path, label):
        if hasattr(self, "set_planned_preview"):
            self.set_planned_preview(path, label)

    def plan_trajectory(self, silent=False):
        try:
            tx = self._read_float(self.target_x, "目标X")
            ty = self._read_float(self.target_y, "目标Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            mode = self.mode_combo.currentText()
            start = (self.cur_x, self.cur_y)
            end = (tx, ty)

            if mode.startswith("G1"):
                path = self.generate_linear_path(self.cur_x, self.cur_y, tx, ty, v, silent=silent)
                label = "G1 直线"
            elif mode.startswith("G2") or mode.startswith("G3"):
                radius = self._read_float(self.radius_r, "圆弧半径", positive=True)
                path = self.generate_arc_path(
                    start[0],
                    start[1],
                    end[0],
                    end[1],
                    radius,
                    clockwise=mode.startswith("G2"),
                    speed_max=v,
                    silent=silent,
                )
                label = "G2 顺圆" if mode.startswith("G2") else "G3 逆圆"
            else:
                raise ValueError(f"不支持的轨迹模式: {mode}")

            if not path:
                self.log_error(f"{label}未生成可发送轨迹")
                return
            self.preview_planned_path(path, label)
            self.load_motion_queue(path)
            self.log_display.append(
                f"<font color='cyan'>{label}规划完成: {len(path)} 点，目标 X={tx:.1f}, Y={ty:.1f}</font>"
            )
        except ValueError as e:
            self.log_error(f"轨迹参数错误: {e}")
        except Exception as e:
            self.log_error(f"轨迹规划错误: {e}")

    def _make_line_segment(self, p0, p1):
        return self.path_planner.line_segment(p0, p1)

    def _make_arc_segment(self, p0, p1, radius, clockwise):
        return self.path_planner.arc_segment(p0, p1, radius, clockwise=clockwise)

    def _offset_point(self, origin, point):
        return (origin[0] + point[0], origin[1] + point[1])

    def _line(self, origin, p0, p1):
        return self._make_line_segment(self._offset_point(origin, p0), self._offset_point(origin, p1))

    def _arc(self, origin, p0, p1, radius, clockwise):
        return self._make_arc_segment(self._offset_point(origin, p0), self._offset_point(origin, p1), radius, clockwise)

    def build_car1_segments(self, x0, y0):
        origin = (x0, y0)
        raw = [
            self._line(origin, (0, 0), (0, 24)),
            self._line(origin, (0, 24), (72, 24)),
            self._line(origin, (72, 24), (72, 48)),
            self._line(origin, (72, 48), (108, 48)),
            self._line(origin, (108, 48), (120, 36)),
            self._line(origin, (120, 36), (120, 0)),
            self._line(origin, (120, 0), (108, 0)),
            self._arc(origin, (108, 0), (84, 0), 12, clockwise=True),
            self._line(origin, (84, 0), (48, 0)),
            self._arc(origin, (48, 0), (24, 0), 12, clockwise=True),
            self._line(origin, (24, 0), (0, 0)),
        ]
        return [segment for segment in raw if segment is not None]

    def build_car2_segments(self, x0, y0):
        origin = (x0, y0)
        raw = [
            self._line(origin, (0, 0), (0, 20)),
            self._line(origin, (0, 20), (40, 20)),
            self._line(origin, (40, 20), (60, 40)),
            self._line(origin, (60, 40), (120, 40)),
            self._line(origin, (120, 40), (140, 20)),
            self._line(origin, (140, 20), (160, 20)),
            self._line(origin, (160, 20), (160, 0)),
            self._line(origin, (160, 0), (140, 0)),
            self._arc(origin, (140, 0), (116, 0), 12, clockwise=True),
            self._line(origin, (116, 0), (44, 0)),
            self._arc(origin, (44, 0), (20, 0), 12, clockwise=True),
            self._line(origin, (20, 0), (0, 0)),
        ]
        return [segment for segment in raw if segment is not None]

    def plan_car_path(self):
        try:
            x0 = self._read_float(self.car_start_x, "小车起始X")
            y0 = self._read_float(self.car_start_y, "小车起始Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path = self.generate_geometry_path(self.build_car1_segments(x0, y0), v, silent_first=True, label="小车轨迹1")
            self.preview_planned_path(path, "小车轨迹1")
            self.load_motion_queue(path)
        except ValueError as e:
            self.log_error(f"小车轨迹1参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹1规划错误: {e}")

    def plan_car2_path(self):
        try:
            x0 = self._read_float(self.car_start_x, "小车起始X")
            y0 = self._read_float(self.car_start_y, "小车起始Y")
            v = self._read_float(self.hw_speed_input, "运行速度", positive=True)
            path = self.generate_geometry_path(self.build_car2_segments(x0, y0), v, silent_first=True, label="小车轨迹2")
            self.preview_planned_path(path, "小车轨迹2")
            self.load_motion_queue(path)
        except ValueError as e:
            self.log_error(f"小车轨迹2参数错误: {e}")
        except Exception as e:
            self.log_error(f"小车轨迹2规划错误: {e}")

    def system_reset(self): 
        if self.board_only_debug:
            # 当前仅连接控制板，没有接 HOME 微动开关与电机，先用软件零点完成串口和队列调试。
            # 后续接入真实硬件后，将 board_only_debug 改为 False，即恢复发送 $H 的真实回零流程。
            self.is_homed = True
            self.home_sensor_triggered = False
            self.point_queue = []
            self.waiting_for_ack = False
            self.motion_preamble_needed = True
            self.cur_x, self.cur_y = self.HOME_X, self.HOME_Y
            self.history_x, self.history_y = [self.cur_x], [self.cur_y]
            self.update_plot()
            self.timeout_timer.stop()
            self.log_display.append(
                "<font color='yellow'>BOARD_ONLY_DEBUG: 未接 HOME 开关，跳过真实 $H，发送 ZERO 作为软件零点。</font>"
            )
            self.send_ascii_line("ZERO", "SOFT_ZERO")
            return
        if self.ser and self.ser.is_open:
            ts = self.get_timestamp(); cmd = "$H"; self.is_homed = False
            self.last_sent_cs = self.calculate_checksum(cmd); self.last_sent_package = cmd
            self.log_display.append(f"<font color='#ffffff'>TX {ts} [HOME] $H</font>")
            self.ser.write((cmd + "\n").encode('ascii')); self.waiting_for_ack = True; self.timeout_timer.start(10000) 
        else: self.log_error("串口未连接")
        
    def emergency_stop(self):
        self.point_queue = []; self.waiting_for_ack = False; self.timeout_timer.stop()
        self.motion_preamble_needed = True
        ts = self.get_timestamp(); self.log_display.append(f"<font color='#e74c3c'>TX {ts} ESTOP (急停)</font>")
        if self.ser and self.ser.is_open: self.ser.write(b"ESTOP\n")
            
    def add_jog(self, dx, dy):
        try:
            v_max = float(self.jog_speed_input.text())
            sx, sy = (self.point_queue[-1][0], self.point_queue[-1][1]) if self.point_queue else (self.cur_x, self.cur_y)
            if not self.check_workspace_safety(sx, sy):
                sx, sy = self.kinematics.find_safe_home((self.HOME_X, self.HOME_Y))
                self.cur_x, self.cur_y = sx, sy
                self.history_x, self.history_y = [sx], [sy]
                self.log_display.append(
                    f"<font color='yellow'>当前位置不可达，已校正到安全点 X={sx:.1f}, Y={sy:.1f}</font>"
                )
                self.update_plot()
            tx, ty = sx + dx, sy + dy
            if self.check_workspace_safety(tx, ty):
                path = self.generate_linear_path(sx, sy, tx, ty, v_max, v_start=0.0, v_end=0.0)
                if not path:
                    self.log_error("点动距离过短，未生成轨迹")
                    return
                self.preview_planned_path(path, "点动")
                self.load_motion_queue(path, append=True)
            else: self.log_error(f"不可达区域: X={tx:.1f}, Y={ty:.1f}")
        except Exception as e: self.log_error(f"点动错误: {e}")
    
    def motor_jog(self, motor_id, direction):
        """
        单独控制电机轴实现正向/逆向旋转5°
        :param motor_id: 电机编号 (1 或 2)
        :param direction: 方向 (1: 正向, -1: 逆向)
        """
        try:
            # 获取当前角度
            current_angles = self.inverse_kinematics(self.cur_x, self.cur_y)
            if current_angles[0] is None or current_angles[1] is None:
                self.log_error("无法获取当前关节角度")
                return
            
            q1, q2 = current_angles
            
            # 单次旋转角度 = 3度
            half_turn = 3.0
            new_q1, new_q2 = q1, q2
            
            if motor_id == 1:
                new_q1 = q1 + direction * half_turn
            elif motor_id == 2:
                new_q2 = q2 + direction * half_turn
            else:
                self.log_error(f"无效电机编号: {motor_id}")
                return
            
            # 使用正向运动学计算新的末端位置
            new_x, new_y = self.kinematics.forward(new_q1, new_q2)
            if new_x is None or new_y is None:
                self.log_error(f"电机{motor_id}旋转半圈后位置不可达")
                return

            # 检查新位置是否安全
            if not self.check_workspace_safety(new_x, new_y):
                self.log_error(f"电机{motor_id}旋转后超出工作空间: X={new_x:.1f}, Y={new_y:.1f}")
                return

            v_max = float(self.jog_speed_input.text())
            path = self.generate_linear_path(self.cur_x, self.cur_y, new_x, new_y, v_max, v_start=0.0, v_end=0.0)

            if not path:
                self.log_error("电机点动距离过短，未生成轨迹")
                return

            self.preview_planned_path(path, f"电机{motor_id}点动")
            self.load_motion_queue(path, append=True)
            self.log_display.append(
                f"<font color='cyan'>电机{motor_id} {direction > 0 and '正向' or '逆向'}旋转半圈: ({self.cur_x:.1f},{self.cur_y:.1f}) -> ({new_x:.1f},{new_y:.1f})</font>"
            )

        except Exception as e:
            self.log_error(f"电机控制错误: {e}")
