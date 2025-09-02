# -*- coding: utf-8 -*-
"""
flight_test_policy.py
- Load trained PPO policy (SB3) and run it inside the original 400 Hz control loop.
- Policy outputs 4x(Δx,Δy,Δz) [mm] at 50 Hz; we apply IK and send joint targets to data.ctrl[8:28].
- Arms are held at center until the FIRST waypoint ([0,0,1]) is reached; then policy is enabled.
"""
import os, sys, time, threading, signal
import math
import numpy as np
import mujoco
import mujoco.viewer

# set GL backend before mujoco import (safety if you move this code around)
os.environ.setdefault("MUJOCO_GL", "glfw")

from controller_geom_wrapper import ControllerGeom
from stable_baselines3 import PPO

# ---------------- Config ----------------
CTRL_HZ = 400.0               # mujoco control Hz = controller Hz
CTRL_DT = 1.0 / CTRL_HZ
VIEWER_HZ = 30.0
POLICY_HZ = 50.0
POLICY_DT = 1.0 / POLICY_HZ
INNER_STEPS = int(CTRL_HZ // POLICY_HZ)  # 8

ARM_XY_CENTER = np.array([290., 0.], dtype=np.float64)  # (x,y) in [mm]
ARM_XY_LIMIT  = np.array([-45., 45.], dtype=np.float64) # (x,y) in [mm]
ARM_Z_CENTER  = 125.                                     # (z) in [mm]
ARM_Z_LIMIT   = np.array([-45., 45.], dtype=np.float64)  # (z) in [mm]

# Per-policy-step EE delta limit [mm] (safe start)
DELTA_STEP_MAX = np.array([5.0, 5.0, 5.0], dtype=np.float64)

PWM_A = 70.      # motor model: thrust[N] = PWM_A * pwm^2 + PWM_B
PWM_B = 8.
PWM_ZETA = 0.03  # Torque[Nm] = PWM_ZETA * thrust[N]
ROTOR_DIR = np.array([+1, -1, +1, -1], dtype=np.float64)

GOAL_TOL = 0.40  # [m], waypoint reach tolerance

TARGETS = [
  np.array([0.0, 0.0, 3.0], dtype=np.float64),
  np.array([6.0, 0.0, 3.0], dtype=np.float64),
  np.array([6.0, 6.0, 3.0], dtype=np.float64),
  np.array([0.0, 6.0, 3.0], dtype=np.float64),
]

# Paths to trained artifacts
HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(HERE, "ppo_arm_reach.zip")
VECNORM_PATH = os.path.join(HERE, "vecnorm.pkl")  # saved by VecNormalize


# -------- Utils: quat(wxyz) -> rpy --------
def quat_wxyz_to_rpy(q):
    """Convert wxyz quaternion to roll-pitch-yaw (XYZ convention)."""
    w, x, y, z = q
    t0 = +2.0*(w*x + y*z)
    t1 = +1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(t0, t1)
    t2 = +2.0*(w*y - z*x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0*(w*z + x*y)
    t4 = +1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(t3, t4)
    return np.array([roll, pitch, yaw], dtype=np.float64)


# -------- IK (same as your original) --------
def ik(var):
    # arm length -> a0, a1, a2, a3, a4 = 134., 115., 110., 24., 104.
    des_x, des_y, des_z, heading = var

    p05 = np.array([[-des_x, des_y, des_z]], dtype=np.float64)
    p04 = p05 - 104.0 * heading

    # theta_1
    th1 = -np.atan2(p04[0,0], p04[0,1]) - np.pi/2

    # theta_5
    n = p04[0,1]*heading[0] - p04[0,0]*heading[1]
    denom = np.sqrt(p04[0,1]**2 + p04[0,0]**2)
    c5 = np.clip(np.abs(n) / (denom + 1e-12), -1.0, 1.0)
    th5 = np.arccos(c5)
    if th5 <= np.pi/2: th5 -= np.pi/2
    if p04[0,0]*p05[0,1] - p04[0,1]*p05[0,0] > 0: th5 = -th5

    # p03
    cos_1 = np.cos(th1)
    sin_1 = np.sin(th1)
    heading_projected = heading - np.sin(th5) * np.array([[sin_1, -cos_1, 0]], dtype=np.float64)
    p34 = 24. * heading_projected / (np.linalg.norm(heading_projected) + 1e-12)
    p03 = p04 - p34

    # theta_2,3
    p01 = np.array([[-134*cos_1, -134*sin_1, 0]], dtype=np.float64)
    x_prime = np.sqrt((p01[0,0]-p03[0,0])**2 + (p01[0,1]-p03[0,1])**2)
    y_prime = p03[0,2]
    xy_sqr_sum = x_prime**2 + y_prime**2

    cos_3 = (xy_sqr_sum - 25325.0) / 25300.0
    cos_3 = float(np.clip(cos_3, -1.0, 1.0))
    sin_3 = np.sqrt(max(0.0, 1.0 - cos_3**2))
    th3 = -np.arccos(cos_3)

    if p03[0,0] < p01[0,0]:
        th2 = - np.arctan2(y_prime, x_prime) + np.arctan2(110*sin_3, 115+110*cos_3)
    else:
        th2 = np.arctan2(y_prime, x_prime) + np.arctan2(110*sin_3, 115+110*cos_3) - np.pi

    # theta_4
    cos_2 = np.cos(th2)
    p02 = p01 - 115 * np.array([[cos_2*cos_1, cos_2*sin_1, np.sin(th2)]], dtype=np.float64)
    p32 = p02 - p03

    cos_4 = np.dot(p32[0], p34[0]) / 2640.0
    cos_4 = float(np.clip(cos_4, -1.0, 1.0))
    th4 = np.arccos(cos_4) if abs(cos_4) != 1 else np.pi

    th4_ref = np.arctan2(p34[0,2], p34[0,0]) - np.arctan2(p32[0,2], p32[0,0])
    if th4_ref < 0: th4_ref += 2*np.pi
    if th4_ref > np.pi: th4 = -th4

    return [th1, -th2, -th3, np.pi-th4, th5]


# -------- inertia in IMU frame --------
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


# -------- VecNormalize stats loader --------
def load_vecnorm_stats(path):
    """Load VecNormalize object and return (mean, var, clip_obs, epsilon),
    or None if unavailable."""
    if not os.path.exists(path):
        return None
    try:
        import cloudpickle
        with open(path, "rb") as f:
            vn = cloudpickle.load(f)
        mean = np.asarray(vn.obs_rms.mean, dtype=np.float64)
        var  = np.asarray(vn.obs_rms.var, dtype=np.float64)
        clip = float(getattr(vn, "clip_obs", 10.0))
        eps  = float(getattr(vn, "epsilon", 1e-8))
        return (mean, var, clip, eps)
    except Exception as e:
        print(f"[warn] VecNormalize stats load failed: {e}")
        return None

def normalize_obs(obs, stats):
    """Apply VecNormalize-like observation normalization if stats available."""
    if stats is None:
        return obs
    mean, var, clip, eps = stats
    obs_n = (obs - mean) / np.sqrt(var + eps)
    obs_n = np.clip(obs_n, -clip, clip)
    return obs_n

# --- 2D overlay text (robust across viewer versions) ---
def _overlay_text(viewer, title, value,
                  grid=mujoco.mjtGridPos.mjGRID_TOPLEFT):
    """Draw overlay text using whatever the viewer provides.

    Tries, in order:
      1) viewer.add_overlay(font, grid, t1, t2)
      2) viewer.add_overlay(grid, t1, t2)
      3) viewer._overlay / viewer.overlay dict (if present)
      4) mujoco.mjr_overlay(...) using discovered context/viewport
    """
    # 1) Newer signatures (with font)
    if hasattr(viewer, "add_overlay"):
        try:
            viewer.add_overlay(mujoco.mjtFont.mjFONT_NORMAL, grid, title, value)
            return
        except TypeError:
            # 2) Older signature (no font)
            try:
                viewer.add_overlay(grid, title, value)
                return
            except Exception:
                pass  # fall through

    # 3) Hidden overlay dict used by some builds
    ov = getattr(viewer, "_overlay", None) or getattr(viewer, "overlay", None)
    if ov is not None:
        try:
            lst = ov.setdefault(grid, [])
            lst.append((title, value))
            return
        except Exception:
            pass

    # 4) Last resort: direct renderer overlay (works if we can find ctx/rect)
    ctx = (getattr(viewer, "context", None) or getattr(viewer, "_context", None) or
           getattr(viewer, "render_context", None) or getattr(viewer, "_render_context", None))
    rect = (getattr(viewer, "viewport", None) or getattr(viewer, "_viewport", None) or
            getattr(viewer, "rect", None) or getattr(viewer, "_rect", None))
    if (ctx is not None) and (rect is not None):
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_NORMAL,  # font
            grid,                          # grid position
            rect,                          # viewport rect
            title,                         # first line
            value,                         # second line
            ctx                            # renderer context
        )
    # If all attempts fail, silently skip (no crash).


# -------- SetPoint drawer ---------------
import numpy as np
import mujoco

def _add_sphere_marker(scn: mujoco.MjvScene, pos, radius=0.08, rgba=(1.0, 0.0, 0.0, 0.95)):
    """Append one sphere marker to user scene (in-place init; no tuple item assignment)."""
    # Prepare buffers with required dtypes/shapes
    size64 = np.array([radius, radius, radius], dtype=np.float64)   # float64[3]
    pos64  = np.asarray(pos, dtype=np.float64).reshape(3,)          # float64[3]
    mat64  = np.eye(3, dtype=np.float64).reshape(9,)                # float64[9]
    rgba32 = np.array(rgba, dtype=np.float32).reshape(4,)           # float32[4]

    # Guard against capacity
    if scn.ngeom >= scn.maxgeom:
        return  # or log a warning

    # In-place initialize the *existing* slot: no assignment to the tuple needed
    mujoco.mjv_initGeom(
        scn.geoms[scn.ngeom],                    # <-- in-place target
        mujoco.mjtGeom.mjGEOM_SPHERE,
        size64, pos64, mat64, rgba32
    )
    scn.geoms[scn.ngeom].category = mujoco.mjtCatBit.mjCAT_DECOR
    scn.ngeom += 1

def _draw_setpoints(viewer, pos_des, targets):
    """Clear user scene and draw setpoint markers every frame."""
    scn = viewer.user_scn
    scn.ngeom = 0  # clear previous frame

    # Current setpoint (red, bigger)
    _add_sphere_marker(scn, pos_des, radius=0.08, rgba=(1, 0, 0, 0.95))

    # All waypoints (gray, semi-transparent)
    for t in targets:
        same = np.allclose(t, pos_des)
        _add_sphere_marker(
            scn, t,
            radius=0.06,
            rgba=(0.5, 0.5, 0.5, 0.40 if same else 0.25)
        )


def main():
    # ---------- Resolve paths ----------
    xml_path = os.path.join(HERE, "xml", "scene.xml")
    if not os.path.exists(xml_path):
        raise FileNotFoundError(f"XML not found: {xml_path}")
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

    # ---------- Load MuJoCo model ----------
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    model.opt.timestep = CTRL_DT
    imu_site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, b"imu")

    # ---------- Controller & Policy ----------
    ctrl = ControllerGeom()
    policy = PPO.load(MODEL_PATH, device="auto")
    vn_stats = load_vecnorm_stats(VECNORM_PATH)  # (mean,var,clip,eps) or None

    # ---------- RL state (EE targets, policy gate) ----------
    heading = np.array([0., 0., 1.], dtype=np.float64)
    ee_mm = np.tile(np.array([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER]), (4, 1)).astype(np.float64)
    joint_targets = np.asarray([ik([ee_mm[i,0], ee_mm[i,1], ee_mm[i,2], heading]) for i in range(4)],
                               dtype=np.float64).reshape(-1)
    last_pwm = np.zeros(4, dtype=np.float64)
    target_idx = 0
    pos_des = TARGETS[target_idx].copy()
    policy_enabled = False  # enable only after first waypoint is reached

    # ---------- Threads ----------
    lock = threading.Lock()
    stop_flag = {"stop": False}

    # ---------- Control loop (400 Hz physics & controller; 50 Hz policy) ----------
    def control_loop():
        nonlocal ee_mm, joint_targets, last_pwm, target_idx, pos_des, policy_enabled

        # CLOSED-LOOP warmup (0.5 s): arms fixed at center, controller active
        for _ in range(int(0.5 * CTRL_HZ)):
            with lock:
                # read state
                pos = np.array(data.qpos[0:3], dtype=np.float64)
                q_wxyz = np.array(data.qpos[3:7], dtype=np.float64)
                lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
                lin_acc = np.array(data.qacc[:3], dtype=np.float64)
                ang_vel = np.array(data.qvel[3:6], dtype=np.float64)
                J_imu = compute_system_inertia_in_imu_frame(model, data, imu_site)
                arm_pos = np.array(data.qpos[7:27], dtype=np.float64)

                ctrl.state_update(pos_des3=pos_des, heading_yaw=0.0, pos3=pos, vel3=lin_vel,
                                  acc3=lin_acc, quat_wxyz4=q_wxyz, gyro3=ang_vel,
                                  inertia=J_imu.reshape(9,), arm_pos=arm_pos)
                _, pwm = ctrl.compute_pwm()
                pwm = np.nan_to_num(pwm, nan=0.0, posinf=1.0, neginf=0.0)
                pwm = np.clip(pwm, 0.0, 1.0)
                last_pwm = pwm

                F = PWM_A * pwm**2 + PWM_B
                Tau = PWM_ZETA * F * ROTOR_DIR

                data.ctrl[0:4] = F
                data.ctrl[4:8] = Tau
                data.ctrl[8:28] = joint_targets
                mujoco.mj_step(model, data)

        # main loop
        next_policy_t = time.perf_counter()
        while not stop_flag["stop"]:
            loop_start = time.perf_counter()
            # ----- policy tick @ 50 Hz -----
            if loop_start >= next_policy_t:
                with lock:
                    # build observation (pos_err, rpy, ang_vel, lin_vel, ee_rel, last_pwm)
                    pos = np.array(data.qpos[0:3], dtype=np.float64)
                    q_wxyz = np.array(data.qpos[3:7], dtype=np.float64)
                    ang_vel = np.array(data.qvel[3:6], dtype=np.float64)
                    lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
                    rpy = quat_wxyz_to_rpy(q_wxyz)
                    pos_err = pos - pos_des
                    ee_rel = ee_mm - np.array([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER])

                    obs = np.concatenate([pos_err, rpy, ang_vel, lin_vel, ee_rel.reshape(-1), last_pwm], dtype=np.float64).astype(np.float32)
                    obs_n = normalize_obs(obs, vn_stats).astype(np.float32)
                    
                    # policy gating: before first waypoint, keep arms at center
                    if policy_enabled:
                        action, _ = policy.predict(obs_n, deterministic=True)
                        action = np.asarray(action, dtype=np.float64).reshape(4, 3)
                        # clamp per-step deltas
                        action = np.clip(action, -DELTA_STEP_MAX, +DELTA_STEP_MAX)
                        # apply to EE mm
                        ee_mm += action
                        # workspace limits
                        xy_min = ARM_XY_CENTER + ARM_XY_LIMIT[0]
                        xy_max = ARM_XY_CENTER + ARM_XY_LIMIT[1]
                        ee_mm[:, 0] = np.clip(ee_mm[:, 0], xy_min[0], xy_max[0])
                        ee_mm[:, 1] = np.clip(ee_mm[:, 1], xy_min[1], xy_max[1])
                        ee_mm[:, 2] = np.clip(ee_mm[:, 2],
                                              ARM_Z_CENTER + ARM_Z_LIMIT[0],
                                              ARM_Z_CENTER + ARM_Z_LIMIT[1])
                    else:
                        # freeze at center
                        ee_mm[:] = np.tile([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER], (4, 1))

                    # IK -> joints (20)
                    joint_targets = np.asarray([ik([ee_mm[i,0], ee_mm[i,1], ee_mm[i,2], heading])
                                                for i in range(4)], dtype=np.float64).reshape(-1)

                    # waypoint switch by reach condition
                    if np.linalg.norm(pos - pos_des) < GOAL_TOL:
                        # enable policy right after finishing FIRST waypoint
                        if not policy_enabled and target_idx == 0:
                            policy_enabled = True
                        # advance waypoint
                        target_idx = (target_idx + 1) % len(TARGETS)
                        pos_des = TARGETS[target_idx].copy()

                next_policy_t += POLICY_DT

            # ----- inner 400 Hz step -----
            with lock:
                pos = np.array(data.qpos[0:3], dtype=np.float64)
                q_wxyz = np.array(data.qpos[3:7], dtype=np.float64)
                lin_vel = np.array(data.qvel[0:3], dtype=np.float64)
                lin_acc = np.array(data.qacc[:3], dtype=np.float64)
                ang_vel = np.array(data.qvel[3:6], dtype=np.float64)
                J_imu = compute_system_inertia_in_imu_frame(model, data, imu_site)
                arm_pos = np.array(data.qpos[7:27], dtype=np.float64)

                ctrl.state_update(pos_des3=pos_des, heading_yaw=0.0, pos3=pos, vel3=lin_vel,
                                  acc3=lin_acc, quat_wxyz4=q_wxyz, gyro3=ang_vel,
                                  inertia=J_imu.reshape(9,), arm_pos=arm_pos)
                _, pwm = ctrl.compute_pwm()
                pwm = np.nan_to_num(pwm, nan=0.0, posinf=1.0, neginf=0.0)
                pwm = np.clip(pwm, 0.0, 1.0)
                last_pwm = pwm

                F = PWM_A * pwm**2 + PWM_B
                Tau = PWM_ZETA * F * ROTOR_DIR

                data.ctrl[0:4] = F
                data.ctrl[4:8] = Tau
                data.ctrl[8:28] = joint_targets

                mujoco.mj_step(model, data)

            # timing to keep ~400 Hz
            loop_end = time.perf_counter()
            sleep_t = CTRL_DT - (loop_end - loop_start)
            if sleep_t > 0:
                time.sleep(sleep_t)

    # ---------- Viewer loop ----------
    def viewer_loop():
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running() and not stop_flag["stop"]:
                time.sleep(1.0 / VIEWER_HZ)
                with lock:
                    # 3D setpoint markers (uncomment to see spheres)
                    _draw_setpoints(viewer, pos_des, TARGETS)

                    # 2D overlay HUD
                    _overlay_text(viewer, "Setpoint [m]",
                                f"x={pos_des[0]:.2f}, y={pos_des[1]:.2f}, z={pos_des[2]:.2f}")
                    _overlay_text(viewer, "Policy", f"enabled={policy_enabled}")
                    _overlay_text(viewer, "Target Idx", f"{target_idx}")

                    viewer.sync()

    # ---------- Start threads ----------
    th_view = threading.Thread(target=viewer_loop, daemon=True)
    th_view.start()
    th_ctrl = threading.Thread(target=control_loop, daemon=True)
    th_ctrl.start()

    # ---------- SIGINT handling ----------
    def _sigint(_sig, _frm):
        stop_flag["stop"] = True
    signal.signal(signal.SIGINT, _sigint)

    print("[flight_test_policy] Started. Close the window or Ctrl+C to exit.")
    try:
        while th_view.is_alive():
            time.sleep(0.2)
    finally:
        stop_flag["stop"] = True
        th_ctrl.join(timeout=1.0)
        try:
            ctrl.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
