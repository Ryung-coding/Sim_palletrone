import os, sys, time, threading, signal
import numpy as np
import mujoco
import mujoco.viewer

from controller_geom_wrapper import ControllerGeom

# ---------- Config ----------
CTRL_HZ = 400.0               # mujoco control Hz = controller Hz
CTRL_DT = 1.0 / CTRL_HZ
VIEWER_HZ = 30.

ARM_XY_CENTER = [290., 0.]    # (x,y) in [mm]
ARM_XY_LIMIT  = [-45., 45.]   # (x,y) in [mm]
ARM_Z_CENTER = 125.           # (z) in [mm]
ARM_Z_LIMIT  = [-45., 45.]    # (z) in [mm]

PWM_A = 70.      # motor model
PWM_B = 8.       # thrust[N] = PWM_A * pwm^2 + PWM_B
PWM_ZETA = 0.03  # Torque[Nm] = PWM_ZETA * thrust[N]

targets = [ # Desired hover setpoint [m]
      np.array([0.0, 0.0, 1.0], dtype=np.float64),
      np.array([2.0, 0.0, 1.0], dtype=np.float64),
      np.array([2.0, 2.0, 1.0], dtype=np.float64),
      np.array([0.0, 2.0, 1.0], dtype=np.float64),
    ]

INIT_EE_POSE = [[ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER, np.array([0,0,1], dtype=np.float64)],
                [ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER, np.array([0,0,1], dtype=np.float64)],
                [ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER, np.array([0,0,1], dtype=np.float64)],
                [ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER, np.array([0,0,1], dtype=np.float64)]]

def ik(var):
  # arm length -> a0, a1, a2, a3, a4 = 134., 115., 110., 24., 104.
  des_x, des_y, des_z, heading = var
  
  p05 = np.array([[-des_x, des_y, des_z]], dtype=np.float64)
  p04 = p05 - 104.0 * heading

  # theta_1
  th1 = -np.atan2(p04[0,0], p04[0,1]) - np.pi/2

  # theta_5
  n = p04[0,1]*heading[0] - p04[0,0]*heading[1]
  th5 = np.arccos(np.abs(n) / np.sqrt(p04[0,1]**2 + p04[0,0]**2))
  if th5 <= np.pi/2: th5 -= np.pi/2
  if p04[0,0]*p05[0,1] - p04[0,1]*p05[0,0] > 0: th5 = -th5

  # p03
  cos_1 = np.cos(th1)
  sin_1 = np.sin(th1)
  heading_projected = heading - np.sin(th5) * np.array([[sin_1, -cos_1, 0]], dtype=np.float64)
  p34 = 24. * heading_projected / np.linalg.norm(heading_projected)
  p03 = p04 - p34

  # theta_2,3
  p01 = np.array([[-134*cos_1, -134*sin_1, 0]], dtype=np.float64)
  x_prime = np.sqrt((p01[0,0]-p03[0,0])**2 + (p01[0,1]-p03[0,1])**2)
  y_prime = p03[0,2]
  xy_sqr_sum = x_prime**2 + y_prime**2

  cos_3 = (xy_sqr_sum - 25325.0) / 25300.0
  sin_3 = np.sqrt(1 - cos_3**2)
  th3 = -np.arccos(cos_3)

  if p03[0,0] < p01[0,0]: th2 = - np.atan2(y_prime, x_prime) + np.atan2(110*sin_3, 115+110*cos_3)
  else: th2 = np.atan2(y_prime, x_prime) + np.atan2(110*sin_3, 115+110*cos_3) - np.pi

  # theta_4
  cos_2 = np.cos(th2)
  p02 = p01 - 115 * np.array([[cos_2*cos_1, cos_2*sin_1, np.sin(th2)]], dtype=np.float64)
  p32 = p02 - p03

  cos_4 = np.dot(p32[0], p34[0]) / 2640.0
  th4 = np.arccos(cos_4) if abs(cos_4) != 1 else np.pi
  
  th4_ref = np.atan2(p34[0,2], p34[0,0]) - np.atan2(p32[0,2], p32[0,0])
  if th4_ref < 0: th4_ref += 2*np.pi
  if th4_ref > np.pi: th4 = -th4

  return [th1, -th2, -th3, np.pi-th4, th5]

def compute_system_inertia_in_imu_frame(model: mujoco.MjModel, data: mujoco.MjData, imu_site_id: int) -> np.ndarray:
  # global CoM
  masses = model.body_mass[1:]         # skip world
  x_coms = data.xipos[1:]
  com_world = np.average(x_coms, axis=0, weights=masses)

  J_world = np.zeros((3, 3))
  I3 = np.eye(3)
  for i in range(1, model.nbody):
    m_i = model.body_mass[i]
    R_i = data.xmat[i].reshape(3, 3)                 # body->world
    I_principal = np.diag(model.body_inertia[i])     # at body CoM, body axes
    I_w = R_i @ I_principal @ R_i.T
    r = data.xipos[i] - com_world
    J_pa = m_i * ((np.dot(r, r) * I3) - np.outer(r, r))
    J_world += I_w + J_pa

  R_imu = data.site_xmat[imu_site_id].reshape(3, 3)  # imu->world
  return R_imu.T @ J_world @ R_imu                   # world->imu

def main():
  # ---------- Resolve paths ----------
  here = os.path.dirname(os.path.abspath(__file__))
  xml_path = os.path.join(here, "xml", "scene.xml")
  if not os.path.exists(xml_path):
    raise FileNotFoundError(f"XML not found: {xml_path}")

  # ---------- Load MuJoCo model ----------
  model = mujoco.MjModel.from_xml_path(xml_path)
  data = mujoco.MjData(model)

  model.opt.timestep = CTRL_DT
  # body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, b"BODY")
  imu_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, b"imu")

  ctrl = ControllerGeom()

  # ---------- Control thread ----------
  lock = threading.Lock()
  stop_flag = {"stop": False}

  ARM_POSE = np.array([ik(INIT_EE_POSE[0]), ik(INIT_EE_POSE[1]), ik(INIT_EE_POSE[2]), ik(INIT_EE_POSE[3])], dtype=np.float64)

  def control_loop():
    for _ in range(int(CTRL_HZ)): # wait for 1sec to run
      with lock:
        data.ctrl[0:8]   = 0.0
        data.ctrl[8:28]  = ARM_POSE.reshape(-1)  # initial arm desired
        mujoco.mj_step(model, data)

    start_time = time.perf_counter()
    rotor_dir = np.array([+1, -1, +1, -1], dtype=np.float64) # Motor spin directions

    while not stop_flag["stop"]:
      next_t = time.perf_counter()

      with lock:
        pos = np.array(data.qpos[0:3], dtype=np.float64)
        q_wxyz = np.array(data.qpos[3:7], dtype=np.float64)
        lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
        lin_acc = np.array(data.qacc[:3], dtype=np.float64)
        ang_vel = np.array(data.qvel[3:6], dtype=np.float64)
        J_imu = compute_system_inertia_in_imu_frame(model, data, imu_site)
        arm_pos = np.array(data.qpos[7:27])

        elapsed = next_t - start_time
        idx = int(elapsed // len(targets)) % len(targets)   # 0,1,2,3 -> repeat
        pos_des = targets[idx]                 # np.array([x,y,z])

        ctrl.state_update(pos_des3=pos_des, heading_yaw=0.0, pos3=pos, vel3=lin_vel, acc3=lin_acc, quat_wxyz4=q_wxyz, gyro3=ang_vel, inertia=J_imu.reshape(9,), arm_pos=arm_pos)
        wrench, pwm = ctrl.compute_pwm()    # (4,) wrench is not used

        F = PWM_A * pwm ** 2 + PWM_B           # (4,)
        Tau = PWM_ZETA * F * rotor_dir         # (4,)

        data.ctrl[0:4] = F
        data.ctrl[4:8] = Tau
        data.ctrl[8:28] = ARM_POSE.reshape(-1)
        
        mujoco.mj_step(model, data)

      # --- timing (400 Hz) ---
      next_t += CTRL_DT
      sleep_t = next_t - time.perf_counter()
      if sleep_t > 0: time.sleep(sleep_t)

  # ---------- Viewer thread ----------
  def viewer_loop():
    with mujoco.viewer.launch_passive(model, data) as viewer:
      while viewer.is_running() and not stop_flag["stop"]:
        time.sleep(1.0 / VIEWER_HZ)
        with lock: viewer.sync()

  # ---------- Start threads ----------
  th_view = threading.Thread(target=viewer_loop, daemon=True)
  th_view.start()
  th_ctrl = threading.Thread(target=control_loop, daemon=True)
  th_ctrl.start()

  # ---------- SIGINT handling ----------
  def _sigint(_sig, _frm):
    stop_flag["stop"] = True
  signal.signal(signal.SIGINT, _sigint)

  print("[run_hover] Started. Close the window or Ctrl+C to exit.")
  try:
    while th_view.is_alive():
      time.sleep(0.2)
  finally:
    stop_flag["stop"] = True
    th_ctrl.join(timeout=1.0)
    ctrl.close()


if __name__ == "__main__":
  main()