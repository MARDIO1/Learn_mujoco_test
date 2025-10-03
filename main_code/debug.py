# osc_plot.py
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
#示波器部分
class Oscilloscope:
    def __init__(self, titles, colors=None, max_points=1000, time_window=None, ylabel="", xlabel="time (s)"):
        """
        titles:  每条曲线名称列表，例如 ["vx","vy","vz"]
        colors:  每条曲线颜色，可选，如 ["C0","C1","C2"]
        max_points: 最大缓存点数（滚动窗口上限）
        time_window: 时间窗口（秒），None 表示用 max_points 控制；两者可二选一
        """
        self.n = len(titles)
        self.max_points = max_points
        self.time_window = time_window
        self.start_time = time.time()

        # 数据缓存
        self.t_buf = deque(maxlen=max_points)
        self.y_bufs = [deque(maxlen=max_points) for _ in range(self.n)]

        # 画布
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 4))
        self.lines = []
        if colors is None:
            colors = [f"C{i}" for i in range(self.n)]
        for i in range(self.n):
            line, = self.ax.plot([], [], color=colors[i], label=titles[i], linewidth=1.4)
            self.lines.append(line)

        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel(ylabel)
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc="upper right")
        self.ax.set_title("Realtime Oscilloscope")

        # 初始范围
        self.ax.set_xlim(0, 1)
        self.ax.set_ylim(-1, 1)

        self.last_draw = 0.0
        self.min_redraw_interval = 0.02  # 50 FPS 上限

    def push(self, t, values):
        """
        追加一帧数据
        t: 时间戳（秒）
        values: 可迭代，长度与 titles 一致
        """
        if not isinstance(values, (list, tuple, np.ndarray)):
            values = [values]
        assert len(values) == self.n, "values length != number of titles"

        self.t_buf.append(t)
        for i, v in enumerate(values):
            self.y_bufs[i].append(float(v))

        # 时间窗口裁剪（按秒）
        if self.time_window is not None and len(self.t_buf) > 2:
            tmax = self.t_buf[-1]
            tmin = tmax - self.time_window
            # 手动裁剪左侧（deque 无按值弹出，只能循环弹）
            while len(self.t_buf) > 0 and self.t_buf[0] < tmin:
                self.t_buf.popleft()
                for q in self.y_bufs:
                    q.popleft()

    def update(self, force=False):
        """
        刷新图像，可在主循环里周期性调用
        force=True 强制刷新（例如步数较小但想立即显示）
        """
        now = time.time()
        if not force and now - self.last_draw < self.min_redraw_interval:
            return

        if len(self.t_buf) < 2:
            return

        t = np.fromiter(self.t_buf, dtype=float)
        for i in range(self.n):
            y = np.fromiter(self.y_bufs[i], dtype=float)
            self.lines[i].set_data(t, y)

        # 自适应 X 轴
        self.ax.set_xlim(t[0], t[-1] if t[-1] > t[0] else t[0] + 1e-3)

        # 自适应 Y 轴（留10%边距）
        ymin = min(np.min(np.fromiter(q, dtype=float)) for q in self.y_bufs)
        ymax = max(np.max(np.fromiter(q, dtype=float)) for q in self.y_bufs)
        if ymin == ymax:
            ymin -= 1.0
            ymax += 1.0
        pad = 0.1 * (ymax - ymin)
        self.ax.set_ylim(ymin - pad, ymax + pad)

        self.fig.canvas.draw_idle()
        plt.pause(0.001)
        self.last_draw = now

    def close(self):
        plt.ioff()
        plt.close(self.fig)
