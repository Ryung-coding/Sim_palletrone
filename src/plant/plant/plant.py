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

def quat_to_euler_zyx(q):  # MuJoCo: [w,x,y,z] -> [roll,pitch,yaw]
    w,x,y,z = q
    yaw   = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
    s     = max(-1.0, min(1.0, 2*(w*y - z*x)))
    pitch = math.asin(s)
    roll  = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    return np.array([roll, pitch, yaw], float)

def sensor_vec(model, data, name):
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name.encode())
    if sid < 0: return None
    adr = model.sensor_adr[sid]; dim = model.sensor_dim[sid]
    return np.array(data.sensordata[adr:adr+dim], float)

class PlantRosNode(Node):
    def __init__(self):
        super().__init__('palletrone_plant')

        pkg_share = get_package_share_directory('plant')
        xml_path  = os.path.join(pkg_share, 'xml', 'scene.xml')

        if not os.path.exists(xml_path):
            self.get_logger().fatal(f"XML not found: {xml_path}")
            sys.exit(1)

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data  = mujoco.MjData(self.model)
        self.model.opt.timestep = 1.0 / PHYSICS_HZ

        self.ctrl = np.zeros(8, float) # order: f1 f2 f3 f4 th1 th2 th3 th4

        self._lock = threading.Lock()
        self._stop = False

        self.prev_t      = None
        self.prev_angvel = None

        self.sub_actuator = self.create_subscription(Input, '/input', self.Actuator_callback, 10)
        self.pub_state = self.create_publisher(PalletroneState, '/palletrone_state', 10)
        self.timer = self.create_timer(1.0 / PHYSICS_HZ, self.publish_state)

        self.viewer_thread = threading.Thread(target=self.viewer_loop, daemon=True)
        self.viewer_thread.start()
        self.sim_thread = threading.Thread(target=self.sim_loop, daemon=True)
        self.sim_thread.start()

    def Actuator_callback(self, msg: Input): # msg.u: [f1 f2 f3 f4 th1 th2 th3 th4]
        with self._lock:
            self.ctrl[:] = np.array(msg.u, float)

    def sim_loop(self):
        next_t = time.perf_counter()
        while rclpy.ok() and not self._stop:
            now = time.perf_counter()
            with self._lock:

                self.data.ctrl[0:4] = self.ctrl[0:4]
                self.data.ctrl[4:8] = self.ctrl[4:8]

                while now >= next_t:
                    mujoco.mj_step(self.model, self.data)
                    next_t += 1.0 / PHYSICS_HZ

            DT = next_t - time.perf_counter()
            if DT > 0: time.sleep(DT)

    def viewer_loop(self):
        try:
            with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                while viewer.is_running() and rclpy.ok() and not self._stop:
                    time.sleep(0.016)
                    with self._lock:
                        viewer.sync()
        except Exception as e:
            self.get_logger().warn(f"viewer end: {e}")

    def publish_state(self):
        with self._lock:
            pos        = sensor_vec(self.model, self.data, "base_pos")      
            linvel     = sensor_vec(self.model, self.data, "base_linvel")  
            linacc     = sensor_vec(self.model, self.data, "base_linacc") 
            quat       = sensor_vec(self.model, self.data, "base_quat") 
            angvel     = sensor_vec(self.model, self.data, "base_angvel") 

            rpy = quat_to_euler_zyx(quat)

            t = time.perf_counter()
            if self.prev_t is not None and self.prev_angvel is not None:
                dt = max(1e-6, t - self.prev_t)
                a_rpy = (angvel - self.prev_angvel) / dt
            else:
                a_rpy = np.zeros(3)
            self.prev_t = t
            self.prev_angvel = angvel.copy()

            if linacc is None:
                if not hasattr(self, "prev_linvel"): self.prev_linvel = linvel.copy()
                dt_lin = max(1e-6, t - getattr(self, "prev_t_lin", t))
                linacc = (linvel - self.prev_linvel) / dt_lin if hasattr(self, "prev_t_lin") else np.zeros(3)
                self.prev_linvel = linvel.copy()
                self.prev_t_lin  = t

        msg = PalletroneState()
        msg.pos   = pos.tolist()
        msg.vel   = linvel.tolist()
        msg.acc   = linacc.tolist()
        msg.rpy   = rpy.tolist()
        msg.w_rpy = angvel.tolist()
        msg.a_rpy = a_rpy.tolist()
        self.pub_state.publish(msg)

    def close(self):
        self._stop = True

def main():
    rclpy.init()
    node = PlantRosNode()
    def _sigint(_s,_f):
        node.close()
    signal.signal(signal.SIGINT, _sigint)
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
