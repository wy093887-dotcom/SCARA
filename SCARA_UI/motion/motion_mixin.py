import numpy as np


class ScaraMotionMixin:
    def inverse_kinematics(self, x, y):
        return self.kinematics.inverse(x, y)

    def check_workspace_safety(self, x, y):
        return self.kinematics.is_reachable(x, y, margin=5.0)

    def generate_linear_path(self, x1, y1, x2, y2, speed_max, v_start=0.0, v_end=0.0, silent=False):
        planned = self.path_planner.plan_line(
            (x1, y1),
            (x2, y2),
            feed_mm_s=speed_max,
            start_speed=v_start,
            end_speed=v_end,
            silent=silent,
        )
        return [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]

    def generate_polyline_path(self, points, speed_max, silent_first=False):
        # 上位机按 GRBL 风格 look-ahead 预先规划速度；下位机只接收带 F 的 G1 点流。
        clean_points = []
        for x, y in points:
            if not self.check_workspace_safety(x, y):
                self.log_error(f"轨迹点超出工作空间: X={x:.1f}, Y={y:.1f}")
                return []
            if not clean_points or np.hypot(x - clean_points[-1][0], y - clean_points[-1][1]) > 0.001:
                clean_points.append((x, y))
        planned = self.path_planner.plan_polyline(clean_points, feed_mm_s=speed_max, silent_first=silent_first)
        return [(p.x, p.y, p.feed_mm_min, p.silent) for p in planned]

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
            self.load_motion_queue(self.generate_polyline_path(pts, v, silent_first=True))

    def plan_trajectory(self, silent=False):
        try:
            tx, ty, v = float(self.target_x.text()), float(self.target_y.text()), float(self.hw_speed_input.text())
            if not self.check_workspace_safety(tx,ty):
                self.log_error("目标不可达"); return
            path = self.generate_linear_path(self.cur_x, self.cur_y, tx, ty, v, silent=silent)
            self.load_motion_queue(path)
        except: self.log_error("参数错误")

    def plan_car_path(self):
        try:
            x0, y0, v = float(self.car_start_x.text()), float(self.car_start_y.text()), float(self.hw_speed_input.text())
            self.load_motion_queue(self.generate_polyline_path([(self.cur_x, self.cur_y), (x0, y0), (x0+160, y0)], v, silent_first=True))
        except: self.log_error("错误")

    def plan_car2_path(self): self.plan_car_path()

    def system_reset(self): 
        if self.board_only_debug:
            # 当前仅连接控制板，没有接 HOME 微动开关与电机，先用软件零点完成串口和队列调试。
            # 后续接入真实硬件后，将 board_only_debug 改为 False，即恢复发送 $H 的真实回零流程。
            self.is_homed = True
            self.home_sensor_triggered = False
            self.point_queue = []
            self.waiting_for_ack = False
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
                self.load_motion_queue(path, append=True)
            else: self.log_error(f"不可达区域: X={tx:.1f}, Y={ty:.1f}")
        except Exception as e: self.log_error(f"点动错误: {e}")
