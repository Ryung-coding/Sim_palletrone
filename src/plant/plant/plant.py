#!/usr/bin/env python3
import os, sys, time, signal, math, threading
import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
from palletrone_interfaces.msg import Input, PalletroneState

PHYSICS_HZ = 400.0
ZETA = 0.02

DELAY_TIME = 0.01
SIG_POS=1e-3; SIG_VEL=1e-3; SIG_GYRO=1e-3; SIG_SERVO=1e-4 # noise

def quat_to_rpy(q):
    w,x,y,z = q
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    s     = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    return np.array([roll, pitch, yaw], float)

class PlantRosNode(Node):
    def __init__(self):
        super().__init__('palletrone_plant')

        pkg_share = get_package_share_directory('plant')
        xml_path  = os.path.join(pkg_share, 'xml', 'scene.xml')

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ

        def sid(name): return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
        self.sid_quat = sid("body_quat")
        self.sid_gyro = sid("body_gyro")
        self.sid_pos  = sid("base_pos")
        self.sid_vel  = sid("base_linvel")
        self.sid_servo_ang = [sid("servo1_angle"), sid("servo2_angle"), sid("servo3_angle"),sid("servo4_angle")]

        self.s_adr = self.model.sensor_adr
        self.s_dim = self.model.sensor_dim

        self.ctrl = np.zeros(8, dtype=float)

        self.ctrl_recv = np.zeros(8, dtype=float)
        self._delay_len = max(1, int(DELAY_TIME * PHYSICS_HZ))
        self._delay_buf = np.zeros((self._delay_len,8), dtype=float)
        self._delay_idx = 0

        self._lock = threading.Lock()
        self._stop = False
        self.prev_linvel_W = None
        self.prev_gyro_I = None
        self.prev_pub_t    = None

        self.sub_input = self.create_subscription(Input, '/input', self.input_callback, 10)
        self.pub_state = self.create_publisher(PalletroneState, '/palletrone_state', 10)

        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True); self.viewer_thread.start()
        self.sim_thread    = threading.Thread(target=self.sim_loop,    daemon=True); self.sim_thread.start()

    def sensing_state(self, sid):
        adr = self.s_adr[sid]; dim = self.s_dim[sid]
        return np.array(self.data.sensordata[adr:adr+dim], dtype=float)

    def _noisy(self, x, s):
        return x + np.random.normal(0.0, s, size=x.shape)

    def _delay_step(self):
        i = self._delay_idx
        self._delay_buf[i] = self.ctrl_recv
        self._delay_idx = (i + 1) % self._delay_len
        return self._delay_buf[self._delay_idx]

    def input_callback(self, msg: Input):
        with self._lock:
            self.ctrl_recv = np.asarray(msg.u, dtype=float)

    def sim_loop(self):
        next_step = time.perf_counter()
        next_pub  = next_step

        while rclpy.ok() and not self._stop:

            now = time.perf_counter()

            with self._lock:
                self.ctrl = self._delay_step()

                self.data.ctrl[0:4] = ZETA * (self.ctrl[0:4]**2)
                self.data.ctrl[4:8] = self.ctrl[4:8]

                while now >= next_step:
                    mujoco.mj_step(self.model, self.data)
                    next_step += 1.0 / PHYSICS_HZ

                while now >= next_pub:
                    quat_imu_W = self.sensing_state(self.sid_quat)
                    gyro_I     = self._noisy(self.sensing_state(self.sid_gyro), SIG_GYRO)
                    pos_W      = self._noisy(self.sensing_state(self.sid_pos),  SIG_POS)
                    linvel_W   = self._noisy(self.sensing_state(self.sid_vel), SIG_VEL)
                    servo = self._noisy(np.array([self.sensing_state(sid)[0] for sid in self.sid_servo_ang], dtype=float), SIG_SERVO)
                    rpy = quat_to_rpy(quat_imu_W)

                    t = now

                    if self.prev_pub_t is None:
                        acc_W = np.zeros(3)
                        a_rpy = np.zeros(3)
                    else:
                        dt = max(1e-6, t - self.prev_pub_t)
                        acc_W = (linvel_W - self.prev_linvel_W) / dt
                        a_rpy = (gyro_I - self.prev_gyro_I) / dt

                    self.prev_pub_t    = t
                    self.prev_linvel_W = linvel_W.copy()
                    self.prev_gyro_I = gyro_I.copy()

                    msg = PalletroneState()
                    msg.pos   = pos_W.tolist()
                    msg.vel   = linvel_W.tolist()
                    msg.acc   = acc_W.tolist()
                    msg.rpy   = rpy.tolist()
                    msg.w_rpy = gyro_I.tolist()
                    msg.a_rpy = a_rpy.tolist()
                    msg.servo = servo.tolist()
                    
                    self.pub_state.publish(msg)

                    next_pub += 1.0 / PHYSICS_HZ

            sleep_t = next_step - time.perf_counter()

            if sleep_t > 0:
                time.sleep(sleep_t)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    with self._lock:
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")

    def close(self):
        self._stop = True

def main():
    rclpy.init()
    node = PlantRosNode()
    signal.signal(signal.SIGINT, lambda *_: node.close())
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
