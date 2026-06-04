import numpy as np
import matplotlib.ticker as ticker


class ScaraPlotMixin:
    def clear_plot_trace(self): 
        self.history_x, self.history_y = [self.cur_x], [self.cur_y]; self.update_plot()
        
    def log_error(self, m): self.log_display.append(f"<font color='red'>[ERR] {m}</font>")

    def precompute_workspace(self):
        xr, yr = np.linspace(75-330, 75+330, 60), np.linspace(0, 480, 60)
        self.ws_x, self.ws_y = [], []
        for xi in xr:
            for yi in yr:
                if self.inverse_kinematics(xi, yi)[0] is not None:
                    self.ws_x.append(xi); self.ws_y.append(yi)

    def update_plot(self, q1_deg=None, q2_deg=None):
        if q1_deg is None:
            ik = self.inverse_kinematics(self.cur_x, self.cur_y)
            (q1_deg, q2_deg) = ik if ik and ik[0] else (90, 90)
        q1, q2 = np.radians(q1_deg), np.radians(q2_deg)
        C1 = [self.L1*np.cos(q1), self.L1*np.sin(q1)]; C2 = [self.L0+self.L1*np.cos(q2), self.L1*np.sin(q2)]
        self.ax.clear(); self.ax.scatter(self.ws_x, self.ws_y, s=1, color='#e5f2ff')
        self.ax.plot(self.history_x, self.history_y, 'b-', alpha=0.3)
        self.ax.plot([0, C1[0], self.cur_x], [0, C1[1], self.cur_y], 'ro-', lw=3)
        self.ax.plot([150, C2[0], self.cur_x], [0, C2[1], self.cur_y], 'bo-', lw=3)
        self.ax.xaxis.set_major_locator(ticker.MultipleLocator(20)); self.ax.yaxis.set_major_locator(ticker.MultipleLocator(20))
        self.ax.tick_params(axis='x', labelrotation=90, labelsize=7); self.ax.tick_params(axis='y', labelsize=7)
        self.ax.grid(True, alpha=0.15); self.ax.set_xlim(-220, 420); self.ax.set_ylim(-20, 480); self.ax.set_aspect('equal')
        self.status_label.setText(f"坐标: X={self.cur_x:.1f}, Y={self.cur_y:.1f}"); self.canvas.draw()
