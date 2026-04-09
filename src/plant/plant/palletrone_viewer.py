#!/usr/bin/env python3
"""
palletrone_viewer.py  –  Real-time 5×3 grid plotter
────────────────────────────────────────────────────
         col 0           col 1           col 2
row 0    x / x_cmd [m]   y / y_cmd [m]   z / z_cmd [m]
row 1    Fx [N]          Fy [N]          Fz [N]
row 2    roll/cmd [deg]  pitch/cmd [deg] yaw/cmd [deg]
row 3    Mx [Nm]         My [Nm]         Mz [Nm]
row 4    Servo θ1‒4 [deg]  Thrust T1‒4 [N]   tip norm [mm]

Cmd.msg:
    float32[3] pos_cmd
    float32[3] att_cmd      ← attitude command (rad)

WINDOW = 10 s
"""

import sys, math, threading
import numpy as np

import rclpy
from rclpy.node import Node
from palletrone_interfaces.msg import PalletroneState, Cmd, Wrench, Input

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

# ─── Config ───
WINDOW_SEC  = 10.0
MAX_SAMPLES = 5000
UPDATE_MS   = 50
RAD2DEG     = 180.0 / math.pi
K_THRUST    = 0.02
TIP_DESIRED = np.array([0.0, -2.0, 2.0])

pg.setConfigOption("background", "w")
pg.setConfigOption("foreground", "k")

C_R, C_G, C_B, C_O = "#e6194b", "#3cb44b", "#4363d8", "#f58231"
C4 = [C_R, C_G, C_B, C_O]


# ─── Ring Buffer ───
class Ring:
    def __init__(self, cap, w):
        self._c, self._w = cap, w
        self._b = np.full((cap, w), np.nan, dtype=np.float64)
        self._i, self._n = 0, 0

    def push(self, row):
        self._b[self._i] = row
        self._i = (self._i + 1) % self._c
        if self._n < self._c: self._n += 1

    def get(self):
        if self._n == 0: return np.empty((0, self._w), dtype=np.float64)
        if self._n < self._c: return self._b[:self._n].copy()
        return np.concatenate([self._b[self._i:], self._b[:self._i]], axis=0)


# ─── ROS 2 Node ───
class VNode(Node):
    def __init__(self):
        super().__init__("palletrone_viewer")
        self.lock = threading.Lock()
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.buf_pos  = Ring(MAX_SAMPLES, 4)   # t x y z
        self.buf_rpy  = Ring(MAX_SAMPLES, 4)   # t r p y  [deg]
        self.buf_srv  = Ring(MAX_SAMPLES, 5)   # t θ1‒4   [deg]
        self.buf_cmd  = Ring(MAX_SAMPLES, 4)   # t x y z  cmd
        self.buf_att  = Ring(MAX_SAMPLES, 4)   # t r p y  att_cmd [deg]
        self.buf_wr   = Ring(MAX_SAMPLES, 7)   # t Fx Fy Fz Mx My Mz
        self.buf_thr  = Ring(MAX_SAMPLES, 5)   # t T1‒4
        self.buf_scmd = Ring(MAX_SAMPLES, 5)   # t θ1‒4 cmd [deg]
        self.buf_tip  = Ring(MAX_SAMPLES, 2)   # t norm [mm]

        self.create_subscription(PalletroneState, "/palletrone_state", self._cb_st, 10)
        self.create_subscription(Cmd,    "/cmd",    self._cb_cm, 10)
        self.create_subscription(Wrench, "/wrench", self._cb_wr, 10)
        self.create_subscription(Input,  "/input",  self._cb_in, 10)

    def _t(self):
        return self.get_clock().now().nanoseconds * 1e-9 - self.t0

    def _cb_st(self, m):
        t = self._t()
        sv = np.array(m.servo[:4])
        if np.all(np.abs(sv) < 2.0): sv *= RAD2DEG
        tip = np.array(m.tip_pos[:3])
        tip_norm = np.linalg.norm(tip - TIP_DESIRED) * 1000.0
        with self.lock:
            self.buf_pos.push([t, m.pos[0], m.pos[1], m.pos[2]])
            self.buf_rpy.push([t, m.rpy[0]*RAD2DEG, m.rpy[1]*RAD2DEG, m.rpy[2]*RAD2DEG])
            self.buf_srv.push([t, sv[0], sv[1], sv[2], sv[3]])
            self.buf_tip.push([t, tip_norm])

    def _cb_cm(self, m):
        t = self._t()
        with self.lock:
            self.buf_cmd.push([t, m.pos_cmd[0], m.pos_cmd[1], m.pos_cmd[2]])
            self.buf_att.push([t,
                               m.att_cmd[0] * RAD2DEG,
                               m.att_cmd[1] * RAD2DEG,
                               m.att_cmd[2] * RAD2DEG])

    def _cb_wr(self, m):
        with self.lock:
            self.buf_wr.push([self._t(), m.force[0], m.force[1], m.force[2],
                              m.moment[0], m.moment[1], m.moment[2]])

    def _cb_in(self, m):
        t = self._t(); u = np.asarray(m.u, dtype=float)
        with self.lock:
            self.buf_thr.push([t, *(K_THRUST * u[:4]**2)])
            self.buf_scmd.push([t, *(u[4:8] * RAD2DEG)])


# ─── Plot helper ───
def _pen(color, w=2, dash=False):
    return pg.mkPen(color=color, width=w,
                    style=QtCore.Qt.DashLine if dash else QtCore.Qt.SolidLine)

def _mkplot(glw, r, c, ylabel):
    p = glw.addPlot(row=r, col=c)
    p.showGrid(x=True, y=True, alpha=0.3)
    p.setLabel("left", ylabel)
    p.getAxis("left").enableAutoSIPrefix(False)
    p.getAxis("bottom").enableAutoSIPrefix(False)
    for a in ("bottom", "left"):
        p.getAxis(a).setPen(pg.mkPen("k"))
        p.getAxis(a).setTextPen(pg.mkPen("k"))
    return p


# ─── Window ───
class Win(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle("Palletrone Viewer")
        self.resize(1600, 950)

        glw = pg.GraphicsLayoutWidget()
        self.setCentralWidget(glw)

        self._cv = {}
        plots = []

        colors3 = [C_R, C_G, C_B]
        pos_lbl = ["x", "y", "z"]
        rpy_lbl = ["roll", "pitch", "yaw"]
        frc_lbl = ["Fx", "Fy", "Fz"]
        trq_lbl = ["Mx", "My", "Mz"]

        # ── Row 0: Position ──
        for c in range(3):
            p = _mkplot(glw, 0, c, f"{pos_lbl[c]} [m]")
            self._cv[f"pos{c}"] = p.plot(pen=_pen(colors3[c]),            name=pos_lbl[c])
            self._cv[f"cmd{c}"] = p.plot(pen=_pen(colors3[c], dash=True), name=f"{pos_lbl[c]}_cmd")
            p.addLegend(offset=(-10, 5))
            if c < 2:
                p.setYRange(-2, 2)
            else:
                p.setYRange(0, 3)
            plots.append(p)

        # ── Row 1: Force ──
        for c in range(3):
            p = _mkplot(glw, 1, c, f"{frc_lbl[c]} [N]")
            self._cv[f"F{c}"] = p.plot(pen=_pen(colors3[c]), name=frc_lbl[c])
            p.addLegend(offset=(-10, 5))
            plots.append(p)

        # ── Row 2: Attitude ──
        for c in range(3):
            p = _mkplot(glw, 2, c, f"{rpy_lbl[c]} [deg]")
            self._cv[f"rpy{c}"]  = p.plot(pen=_pen(colors3[c]),            name=rpy_lbl[c])
            self._cv[f"acmd{c}"] = p.plot(pen=_pen(colors3[c], dash=True), name=f"{rpy_lbl[c]}_cmd")
            p.addLegend(offset=(-10, 5))
            p.setYRange(-25, 25)
            plots.append(p)

        # ── Row 3: Torque ──
        for c in range(3):
            p = _mkplot(glw, 3, c, f"{trq_lbl[c]} [Nm]")
            self._cv[f"M{c}"] = p.plot(pen=_pen(colors3[c]), name=trq_lbl[c])
            p.addLegend(offset=(-10, 5))
            plots.append(p)

        # ── Row 4: Servo + Thrust + Tip Norm ──
        p_srv = _mkplot(glw, 4, 0, "Servo [deg]")
        for i, cl in enumerate(C4):
            self._cv[f"sv{i}"] = p_srv.plot(pen=_pen(cl),                  name=f"θ{i+1}")
            self._cv[f"sc{i}"] = p_srv.plot(pen=_pen(cl, w=1, dash=True),  name=f"θ{i+1}_cmd")
        p_srv.addLegend(offset=(-10, 5))
        p_srv.setYRange(-40, 40)
        plots.append(p_srv)

        p_thr = _mkplot(glw, 4, 1, "Thrust [N]")
        for i, cl in enumerate(C4):
            self._cv[f"th{i}"] = p_thr.plot(pen=_pen(cl), name=f"T{i+1}")
        p_thr.addLegend(offset=(-10, 5))
        p_thr.setYRange(0, 40)
        plots.append(p_thr)

        p_tip = _mkplot(glw, 4, 2, "tip norm [mm]")
        self._cv["tip_norm"] = p_tip.plot(pen=_pen("#e6194b"),               name="norm")
        self._cv["tip_des"]  = p_tip.plot(pen=_pen("#3cb44b", dash=True),    name="desired")
        p_tip.addLegend(offset=(-10, 5))
        plots.append(p_tip)

        # ── X축 링크 ──
        self._plots = plots
        for p in plots[1:]:
            p.setXLink(plots[0])

        for p in plots:
            p.hideAxis("bottom")
        p_srv.showAxis("bottom"); p_srv.setLabel("bottom", "time [s]")
        p_thr.showAxis("bottom"); p_thr.setLabel("bottom", "time [s]")
        p_tip.showAxis("bottom"); p_tip.setLabel("bottom", "time [s]")

        # ── Timer ──
        self._timer = QtCore.QTimer()
        self._timer.timeout.connect(self._upd)
        self._timer.start(UPDATE_MS)

    def _upd(self):
        nd = self.node
        with nd.lock:
            dp = nd.buf_pos.get();  dr = nd.buf_rpy.get()
            dc = nd.buf_cmd.get();  dw = nd.buf_wr.get()
            dt = nd.buf_thr.get();  dsc = nd.buf_scmd.get()
            dsv = nd.buf_srv.get(); da = nd.buf_att.get()
            dtp = nd.buf_tip.get()

        tn = 0.0
        for d in (dp, dc, dw, dt):
            if d.shape[0]: tn = max(tn, d[-1, 0])
        if tn == 0.0: return
        tl = tn - WINDOW_SEC

        def tr(a):
            return a[a[:, 0] >= tl] if a.shape[0] else a

        dp = tr(dp); dr = tr(dr); dc = tr(dc); dw = tr(dw)
        dt = tr(dt); dsc = tr(dsc); dsv = tr(dsv); da = tr(da)
        dtp = tr(dtp)

        cv = self._cv

        if dp.shape[0]:
            for i in range(3): cv[f"pos{i}"].setData(dp[:, 0], dp[:, 1+i])
        if dc.shape[0]:
            for i in range(3): cv[f"cmd{i}"].setData(dc[:, 0], dc[:, 1+i])

        if dw.shape[0]:
            for i in range(3): cv[f"F{i}"].setData(dw[:, 0], dw[:, 1+i])

        if dr.shape[0]:
            for i in range(3): cv[f"rpy{i}"].setData(dr[:, 0], dr[:, 1+i])
        if da.shape[0]:
            for i in range(3): cv[f"acmd{i}"].setData(da[:, 0], da[:, 1+i])

        if dw.shape[0]:
            for i in range(3): cv[f"M{i}"].setData(dw[:, 0], dw[:, 4+i])

        if dsv.shape[0]:
            for i in range(4): cv[f"sv{i}"].setData(dsv[:, 0], dsv[:, 1+i])
        if dsc.shape[0]:
            for i in range(4): cv[f"sc{i}"].setData(dsc[:, 0], dsc[:, 1+i])

        if dt.shape[0]:
            for i in range(4): cv[f"th{i}"].setData(dt[:, 0], dt[:, 1+i])

        if dtp.shape[0]:
            cv["tip_norm"].setData(dtp[:, 0], dtp[:, 1])
            cv["tip_des"].setData([dtp[0, 0], dtp[-1, 0]], [0.0, 0.0])

        self._plots[0].setXRange(tl, tn, padding=0)


# ─── Entry ───
def main():
    rclpy.init()
    node = VNode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    app = QtWidgets.QApplication(sys.argv)
    win = Win(node)
    win.show()
    code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(code)

if __name__ == "__main__":
    main()