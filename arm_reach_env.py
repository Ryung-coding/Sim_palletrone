import os
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import mujoco
os.environ.setdefault("MUJOCO_GL", "glfw")
from stable_baselines3.common.callbacks import BaseCallback

from controller_geom_wrapper import ControllerGeom   # your existing wrapper
from math import atan2

# ----------------------- Constants -----------------------
CTRL_HZ = 400.0
DT_CTRL = 1.0 / CTRL_HZ
POLICY_HZ = 50.0
INNER_STEPS = int(CTRL_HZ // POLICY_HZ)  # 8

ARM_XY_CENTER = np.array([290., 0.], dtype=np.float64)  # mm
ARM_XY_LIMIT  = np.array([-45., 45.], dtype=np.float64) # mm
ARM_Z_CENTER  = 125.                                    # mm
ARM_Z_LIMIT   = np.array([-45., 45.], dtype=np.float64) # mm

PWM_A = 70.0
PWM_B = 8.0
PWM_ZETA = 0.03
ROTOR_DIR = np.array([+1, -1, +1, -1], dtype=np.float64)

GOAL_TOL = 0.3  # m
TARGETS = [
  np.array([0.0, 0.0, 1.0], dtype=np.float64),
  np.array([3.0, 0.0, 1.0], dtype=np.float64),
  np.array([3.0, 3.0, 1.0], dtype=np.float64),
  np.array([0.0, 3.0, 1.0], dtype=np.float64),
]

# Per-step EE delta limits [mm] for numerical stability (tunable)
DELTA_STEP_MAX = np.array([5.0, 5.0, 5.0], dtype=np.float64)

# ----------------------- Utilities -----------------------
def quat_wxyz_to_rpy(q):
    """Convert wxyz quaternion to roll-pitch-yaw (XYZ convention)."""
    w, x, y, z = q
    # roll (x-axis rotation)
    t0 = +2.0*(w*x + y*z)
    t1 = +1.0 - 2.0*(x*x + y*y)
    roll = math.atan2(t0, t1)
    # pitch (y-axis rotation)
    t2 = +2.0*(w*y - z*x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    # yaw (z-axis rotation)
    t3 = +2.0*(w*z + x*y)
    t4 = +1.0 - 2.0*(y*y + z*z)
    yaw = math.atan2(t3, t4)
    return np.array([roll, pitch, yaw], dtype=np.float64)

def ik_single_arm(des_x, des_y, des_z, heading):
  # arm length -> a0, a1, a2, a3, a4 = 134., 115., 110., 24., 104.
  
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

def compute_system_inertia_in_imu_frame(model, data, imu_site_id):
    """Same as your function; rewritten locally to avoid cross-imports."""
    masses = model.body_mass[1:]
    x_coms = data.xipos[1:]
    com_world = np.average(x_coms, axis=0, weights=masses)

    J_world = np.zeros((3, 3))
    I3 = np.eye(3)
    for i in range(1, model.nbody):
        m_i = model.body_mass[i]
        R_i = data.xmat[i].reshape(3, 3)
        I_principal = np.diag(model.body_inertia[i])
        I_w = R_i @ I_principal @ R_i.T
        r = data.xipos[i] - com_world
        J_pa = m_i * ((np.dot(r, r) * I3) - np.outer(r, r))
        J_world += I_w + J_pa

    R_imu = data.site_xmat[imu_site_id].reshape(3, 3)
    return R_imu.T @ J_world @ R_imu

# ----------------------- Gym Env -----------------------
class ArmReachEnv(gym.Env):
    """PPO controls 4x(Δx,Δy,Δz) [mm] for arm EE; flight controller runs at 400Hz."""
    metadata = {"render_modes": ["human", "none"], "render_fps": POLICY_HZ}

    def __init__(self, xml_path: str, render_mode: str | None = None):
        super().__init__()
        assert os.path.exists(xml_path), f"XML not found: {xml_path}"
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = DT_CTRL

        self.ctrl = ControllerGeom()
        self.render_mode = render_mode
        self.viewer = None
        self.imu_site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, b"imu")

        # Action: 4 arms x (dx, dy, dz) [mm per policy step]
        self.action_space = spaces.Box(
            low=np.tile(-DELTA_STEP_MAX, 4),
            high=np.tile(+DELTA_STEP_MAX, 4),
            dtype=np.float32
        )

        # Observation space (set conservative bounds; VecNormalize will help)
        obs_dim = 3 + 3 + 3 + 3 + 12 + 4  # pos_err + rpy + ang_vel + lin_vel + 4*ee_xyz + 4*duty
        high = np.inf * np.ones(obs_dim, dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        # EE commanded positions [mm], initialized at centers
        self.ee_mm = np.tile(np.array([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER]),
                             (4, 1)).astype(np.float64)
        self.heading = np.array([0., 0., 1.], dtype=np.float64)  # fixed heading vector

        self.target_idx = 0
        self.pos_des = TARGETS[self.target_idx].copy()
        self.segment_times = []
        self.elapsed_in_segment = 0.0

        self.center_mm = np.array([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER], dtype=np.float64)
        self.policy_enabled = False  # RL actions apply only after first setpoint is reached

    def _close_viewer(self):
        if self.viewer is not None:
            try:
                self.viewer.close()
            except Exception:
                pass
            self.viewer = None

    def render(self):
        """Human viewer sync; avoid during parallel training."""
        if self.render_mode != "human":
            return
        if self.viewer is None:
            # Open passive viewer window once
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
        if self.viewer.is_running():
            self.viewer.sync()

    def _command_arms_and_step_controller(self, joint_targets_flat):
        """One inner 400Hz step: set controls and step physics once."""
        # 1) Read state
        pos = np.array(self.data.qpos[0:3], dtype=np.float64)
        q_wxyz = np.array(self.data.qpos[3:7], dtype=np.float64)
        lin_vel = np.array(self.data.qvel[0:3], dtype=np.float64)
        lin_acc = np.array(self.data.qacc[0:3], dtype=np.float64)
        ang_vel = np.array(self.data.qvel[3:6], dtype=np.float64)
        J_imu = compute_system_inertia_in_imu_frame(self.model, self.data, self.imu_site)
        arm_pos = np.array(self.data.qpos[7:27], dtype=np.float64)

        # 2) Update flight controller (desired position is self.pos_des, heading_yaw=0)
        self.ctrl.state_update(pos_des3=self.pos_des, heading_yaw=0.0,
                               pos3=pos, vel3=lin_vel, acc3=lin_acc,
                               quat_wxyz4=q_wxyz, gyro3=ang_vel,
                               inertia=J_imu.reshape(9,), arm_pos=arm_pos)
        wrench, pwm = self.ctrl.compute_pwm()  # pwm in [0,1], shape (4,)

        # 3) Convert pwm -> thrust/torque, apply to data.ctrl
        F = PWM_A * (pwm ** 2) + PWM_B   # (4,)
        Tau = PWM_ZETA * F * ROTOR_DIR   # (4,)

        self.data.ctrl[0:4] = F
        self.data.ctrl[4:8] = Tau
        self.data.ctrl[8:28] = joint_targets_flat  # 20 arm joints

        # 4) Step physics
        mujoco.mj_step(self.model, self.data)

        return pwm  # return for reward calc

    def _build_observation(self, last_pwm):
        pos = np.array(self.data.qpos[0:3], dtype=np.float64)
        q_wxyz = np.array(self.data.qpos[3:7], dtype=np.float64)
        ang_vel = np.array(self.data.qvel[3:6], dtype=np.float64)
        lin_vel = np.array(self.data.qvel[0:3], dtype=np.float64)
        rpy = quat_wxyz_to_rpy(q_wxyz)
        pos_err = pos - self.pos_des

        # EE xyz relative to centers [mm]
        ee_rel = self.ee_mm - np.array([ARM_XY_CENTER[0], ARM_XY_CENTER[1], ARM_Z_CENTER])

        obs = np.concatenate([
            pos_err,            # 3
            rpy,                # 3
            ang_vel,            # 3
            lin_vel,            # 3
            ee_rel.reshape(-1), # 12
            last_pwm.astype(np.float64)  # 4
        ], dtype=np.float64)
        return obs.astype(np.float32)

    def _advance_target_if_reached(self):
        pos = np.array(self.data.qpos[0:3], dtype=np.float64)
        if np.linalg.norm(pos - self.pos_des) < GOAL_TOL:
            self.segment_times.append(self.elapsed_in_segment)
            self.elapsed_in_segment = 0.0
            self.target_idx = (self.target_idx + 1) % len(TARGETS)
            self.pos_des = TARGETS[self.target_idx].copy()
            return True
        return False

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.model.opt.timestep = DT_CTRL

        # --- Base pose: start below the first waypoint so it ascends in z ---
        self.data.qpos[0:3] = np.array([0., 0., 0.6])   # <--- start below 1.0 m
        self.data.qpos[3:7] = np.array([1., 0., 0., 0.])  # wxyz

        # Targets & controller state
        self.ctrl = ControllerGeom()
        self.ee_mm[:] = np.tile(self.center_mm, (4, 1))  # start arms at center
        self.target_idx = 0
        self.pos_des = TARGETS[self.target_idx].copy()
        self.segment_times = []
        self.elapsed_in_segment = 0.0
        self.policy_enabled = False                      # <--- lock policy before first reach

        # IK -> joint targets and initialize arm joints to avoid spikes
        jt = []
        for i in range(4):
            th = ik_single_arm(self.ee_mm[i,0], self.ee_mm[i,1], self.ee_mm[i,2], self.heading)
            jt.append(th)
        self._joint_targets_flat = np.asarray(jt, dtype=np.float64).reshape(-1)
        self.data.qpos[7:27] = self._joint_targets_flat
        mujoco.mj_forward(self.model, self.data)

        # --- CLOSED-LOOP WARMUP (0.5 s @ 400 Hz) with arms fixed at center ---
        for _ in range(int(0.5 * CTRL_HZ)):
            _ = self._command_arms_and_step_controller(self._joint_targets_flat)

        last_pwm = np.zeros(4, dtype=np.float64)
        obs = self._build_observation(last_pwm)
        info = {}
        return obs, info

    def step(self, action):
        # --- If policy is disabled (before first waypoint), ignore action and keep arms at center ---
        if not self.policy_enabled:
            # freeze EE at exact center
            self.ee_mm[:] = np.tile(self.center_mm, (4, 1))
        else:
            # apply RL deltas (mm) with clamp
            action = np.asarray(action, dtype=np.float64).reshape(4, 3)
            action = np.clip(action, -DELTA_STEP_MAX, +DELTA_STEP_MAX)
            self.ee_mm += action
            # workspace limits
            xy_min = ARM_XY_CENTER + ARM_XY_LIMIT[0]
            xy_max = ARM_XY_CENTER + ARM_XY_LIMIT[1]
            self.ee_mm[:, 0] = np.clip(self.ee_mm[:, 0], xy_min[0], xy_max[0])
            self.ee_mm[:, 1] = np.clip(self.ee_mm[:, 1], xy_min[1], xy_max[1])
            self.ee_mm[:, 2] = np.clip(self.ee_mm[:, 2],
                                    ARM_Z_CENTER + ARM_Z_LIMIT[0],
                                    ARM_Z_CENTER + ARM_Z_LIMIT[1])

        # IK -> 20 joint targets
        jt = []
        for i in range(4):
            th = ik_single_arm(self.ee_mm[i,0], self.ee_mm[i,1], self.ee_mm[i,2], self.heading)
            jt.append(th)
        self._joint_targets_flat = np.asarray(jt, dtype=np.float64).reshape(-1)

        # 400 Hz inner steps
        reward = 0.0
        reached = False
        last_pwm = np.zeros(4, dtype=np.float64)
        pre_idx = self.target_idx

        for _ in range(INNER_STEPS):
            last_pwm = self._command_arms_and_step_controller(self._joint_targets_flat)

            # If policy is enabled, accumulate time & duty variance penalties
            if self.policy_enabled:
                reward += -(self._alpha_time * DT_CTRL) - (self._beta_var * float(np.var(last_pwm)))

            self.elapsed_in_segment += DT_CTRL

            # Check if the current waypoint is reached
            if self._advance_target_if_reached():
                reward += self._gamma_goal
                reached = True

                # If we just finished the FIRST waypoint (idx 0 -> 1), enable policy now
                if (not self.policy_enabled) and (pre_idx == 0):
                    self.policy_enabled = True

                pre_idx = self.target_idx

        terminated = (len(self.segment_times) == len(TARGETS))
        truncated = False
        if (self.data.qpos[2] < 0.2) or (not np.isfinite(self.data.qpos).all()):
            truncated = True

        obs = self._build_observation(last_pwm)
        info = {
            "target_idx": self.target_idx,
            "segment_times": list(self.segment_times),
            "reached_in_this_step": reached,
            "policy_enabled": self.policy_enabled
        }
        if self.render_mode == "human":
            self.render()
        return obs, float(reward), bool(terminated), bool(truncated), info

    # Tunable reward weights (expose as properties)
    _alpha_time = 10.0   # time penalty [reward/sec]
    _beta_var  = 1.0    # duty variance penalty weight
    _gamma_goal = 100.0  # goal bonus

    def close(self):
        self._close_viewer()
        try:
            self.ctrl.close()
        except Exception:
            pass


class LiveViewerCallback(BaseCallback):
    """
    Periodically runs evaluation episodes in a single env while showing a MuJoCo viewer window.
    Training pauses during evaluation (safe), then resumes.
    """
    def __init__(self, eval_env, eval_freq:int=50_000, n_eval_episodes:int=1, viewer_fps:float=30.0):
        super().__init__()
        self.eval_env = eval_env              # a single ArmReachEnv(render_mode="human")
        self.eval_freq = int(eval_freq)
        self.n_eval_episodes = int(n_eval_episodes)
        self.viewer_fps = float(viewer_fps)

    def _run_one_episode(self):
        # Reset eval env
        obs, _ = self.eval_env.reset()
        # Open viewer as passive window
        with mujoco.viewer.launch_passive(self.eval_env.model, self.eval_env.data) as viewer:
            ep_ret, ep_len = 0.0, 0
            while viewer.is_running():
                # Policy action (deterministic for evaluation)
                action, _ = self.model.predict(obs, deterministic=True)
                obs, reward, terminated, truncated, info = self.eval_env.step(action)
                ep_ret += float(reward); ep_len += 1

                # Pace the viewer
                time.sleep(1.0 / self.viewer_fps)
                # Sync current sim state to the window
                self.eval_env.render()  # calls viewer.sync() internally

                if terminated or truncated:
                    break
        return {"return": ep_ret, "length": ep_len}

    def _on_step(self) -> bool:
        if self.eval_freq > 0 and (self.num_timesteps % self.eval_freq == 0):
            # run n_eval_episodes and log results
            rets, lens = [], []
            for _ in range(self.n_eval_episodes):
                out = self._run_one_episode()
                rets.append(out["return"]); lens.append(out["length"])
            # log to SB3's logger (shows up in TensorBoard)
            self.logger.record("eval/return_mean", np.mean(rets))
            self.logger.record("eval/length_mean", np.mean(lens))
        return True