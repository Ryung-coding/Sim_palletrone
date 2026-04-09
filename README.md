
</head>
<body>

<h1>Palletrone Simulator</h1>
<p>
  If you want to see more information? 
  <a href="https://www.notion.so/ryung-lab/2024-ICT-49897feb72d244a6b8c653a46413805b" target="_blank" rel="noopener">💪 Notion</a><br>
  Video ➡️ 
  <a href="https://youtu.be/9ugz9QTPFvg" target="_blank" rel="noopener"> YouTube</a>
</p>


<p>
  <a href="https://youtu.be/9ugz9QTPFvg" target="_blank" rel="noopener">
    <img src="https://img.youtube.com/vi/9ugz9QTPFvg/hqdefault.jpg" alt="Palletrone Demo on YouTube" width="400">
  </a>
</p>


<section>
  <p>
    This project is developed by <a href="https://mrl.seoultech.ac.kr/index.do" target="_blank" rel="noopener">SeoulTech MRL</a>
  </p>

  <p>
    <strong>Palletrone</strong> is a compound of <em>Pallet</em> and <em>Drone</em>, designed as a fully-actuated multirotor platform.
    It enables stable cargo transportation without undesired attitude changes, overcoming the limitations of underactuated multirotors.
  </p>
</section>

<h2>System Configuration</h2>

<section>
  <div class="figure">
    <img src="https://github.com/user-attachments/assets/91fcdd08-1d03-40fe-806c-e38a8c415a63" alt="Image"  width="600">

  </div>

  <h3>Parameters</h3>
  <table>
    <thead>
      <tr><th>Symbol</th><th>Variable</th><th>Description</th></tr>
    </thead>
    <tbody>
      <tr><td><strong>m</strong></td><td><code>m</code></td><td><strong>Palletrone mass</strong></td></tr>
      <tr><td><strong>ζ</strong></td><td><code>zeta</code></td><td>Force-to-yaw torque transform ratio</td></tr>
      <tr><td><strong>r</strong></td><td><code>r</code></td><td><strong>Moment arm</strong> (Actual hardware arm length = √2·<em>r</em>)</td></tr>
      <tr><td><strong>r<sub>z</sub></strong></td><td><code>r_z</code></td><td><strong>z-axis distance</strong> between IMU (center) and thruster</td></tr>
    </tbody>
  </table>
</section>



<h2>Environment Setup</h2>

<section>
  <p class="muted">
    Tested on <strong>Ubuntu 22.04</strong> with <strong>ROS2 Humble</strong> and <strong>MuJoCo</strong>.
  </p>

  <h3>1) ROS2 Humble Installation</h3>
  <p>Official guide:
    <a href="https://docs.ros.org/en/humble/Installation.html" target="_blank" rel="noopener">ROS2 Humble Installation</a>
  </p>
  <pre><code># Create workspace / Build and source
mkdir -p ~/palletrone_ws/src
cd ~/palletrone_ws 
colcon build --symlink-install
source install/setup.bash
</code></pre>

  <h3>2) MuJoCo Installation</h3>
  <p>Official GitHub:
    <a href="https://github.com/google-deepmind/mujoco" target="_blank" rel="noopener">https://github.com/google-deepmind/mujoco</a>
  </p>
  <pre><code># System dependencies
sudo apt update
sudo apt install libglfw3 libglfw3-dev libglew-dev
python -m pip install mujoco
python -m mujoco.viewer
</code></pre>
</section>



<h2>Launch</h2>

<section>
  <pre><code># Build and source (from your workspace root)
ros2 launch palletrone_cmd pt_launch.py
</code></pre>
</section>



<h2>Project Structure</h2>
<img src="https://github.com/user-attachments/assets/37f021dd-1094-4c5a-b9f0-36eca17d0ba3" alt="Image"  width="1000">


<section>
  <pre><code>Sim_palletrone/
└── src/
    ├── palletrone_interfaces/          # ROS2 message definitions
    │   └── msg/
    │       ├── Cmd.msg                 # Desired position/attitude command (units below)
    │       ├── Input.msg               # Control input: motor speeds & servo angles
    │       ├── PalletroneState.msg     # Simulator state (Z-up)
    │       └── Wrench.msg              # Body-frame forces & torques
    │
    ├── palletrone_cmd/                 # Command publisher
    │   ├── launch/
    │   │   └── pt_launch.py            # Launch plant + controllers + cmd
    │   └── src/
    │       └── position_cmd.cpp        # Publishes 3D desired position
    │
    ├── palletrone_controller/          # Controllers
    │   └── src/
    │       ├── allocator_controller.cpp# Control allocation (ref1-based)
    │       └── wrench_controller.cpp   # Position/attitude PID → Wrench
    │
    └── plant/                          # MuJoCo plant
        ├── plant/
        │   └── plant.py                # Dynamics integration + state publishing
        └── xml/                        # Model & scene (used by the plant)
            ├── Palletrone.xml
            └── scene.xml
</code></pre>
</section>



<h2>Message Definitions</h2>

<section>
  <h3><code>palletrone_interfaces/msg/Cmd.msg</code></h3>
  <pre><code># Desired position command
# Units: position in meters [m]
float32[3] pos_cmd   # [x, y, z] in meters (Z-up world frame)
</code></pre>
  <p><b>Units:</b> position <code>x</code>, <code>y</code>, <code>z</code> are in <b>meters</b> (m).</p>
</section>

<section>
  <h3><code>palletrone_interfaces/msg/Input.msg</code></h3>
  <pre><code># Control input: motor speeds and servo angles
# Units: motor speeds in rad/s, servo angles in rad
    
float32[8] u # omega(u1~u4) : motor BLDC speeds [rad/s] / theta(u5~u8) : servo tilt angles [rad]
</code></pre>
  <p><b>Units:</b> <code>omega</code> in <b>rad/s</b>, <code>theta</code> in <b>rad</b>.</p>
</section>

<section>
  <h3><code>palletrone_interfaces/msg/PalletroneState.msg</code></h3>
  <pre><code># Simulator state (Z-up convention)
# Units: position [m], velocity [m/s], rpy [rad], angular velocity [rad/s]
    
float32[3] pos        # [x, y, z] (Z-up world)
float32[3] vel        # [vx, vy, vz]
float32[3] rpy        # [roll, pitch, yaw]
float32[3] w_rpy      # [roll_rate, pitch_rate, yaw_rate] </code></pre>
  <p><b>Convention:</b> system uses <b>Z-up</b>. Units as indicated in the fields.</p>
</section>

<section>
  <h3>palletrone_interfaces/msg/Wrench.msg</h3>
  <pre><code># Body-frame wrench command
# Units: forces [N], torques [N·m]
    
float32[3] force_N        # [F_x, F_y, F_z] in body frame
float32[3] torque_Nm      # [tau_roll, tau_pitch, tau_yaw] in body frame
</code></pre>
  <p><b>Units:</b> forces in <b>Newtons</b> [N], torques in <b>Newton-meters</b> [N·m], all in the <b>body frame</b>.</p>
</section>



<h2>Node Documentation</h2>

<section>
  <h3><code>palletrone_cmd/src/position_cmd.cpp</code></h3>
  <p>Publishes a <b>3D desired position</b> (<code>Cmd.msg</code>).</p>
  <p><b>Current reference trajectory (mathematical form)</b>:</p>
  <pre><code>ω = 2π / 40
x(t) = 2 · sin(ω t)
y(t) = sin(ω t) · cos(ω t)
z(t) = 5 + 0.5 · sin(ω t + π/2)
</code></pre>
  <p><b>Note:</b> control UI and interfaces are planned to be released later.</p>
</section>

<section>
  <h3><code>palletrone_controller/src/wrench_controller.cpp</code></h3>

  <p>
    Our <strong>Palletrone</strong> is a fully-actuated system.  
    Thus, translational forces (<code>F_x, F_y, F_z</code>) for position control and
    rotational moments (<code>τ_roll, τ_pitch, τ_yaw</code>) for attitude control
    can be regulated independently.
  </p>

  <p>
    The <b>position controller</b> and <b>attitude controller</b> are arranged in parallel,
    each implemented with a single PID loop.
  </p>

  <p>
    - The <b>position controller</b> compares desired position from <code>position_cmd</code>
      with the current state from <code>plant.py</code>, and generates translational forces
      <code>F_x, F_y, F_z</code>. 
      Note: the <b>z-axis includes a feed-forward gravity compensation term</b>.
  </p>

  <p>
    - The <b>attitude controller</b> has desired values fixed at zero (level attitude),
      and compares state feedback from <code>plant.py</code> to generate the rotational moments
      <code>τ_roll, τ_pitch, τ_yaw</code>.
  </p>
  <h4>Control structure (algorithm)</h4>
  <pre><code>1. Define parameters:
   KP, KI, KD gains for each axis
   Integral min/max (anti-windup)
   Output min/max (saturation)
2. Position controller:
   e_pos = pos_ref - pos
   PID → [F_x, F_y, F_z]
   F_z includes + m·g feed-forward term
3. Attitude controller:
   e_att = att_ref(=0) - rpy
   PID → [τ_roll, τ_pitch, τ_yaw]
4. Apply anti-windup & saturation at each PID
5. Publish Wrench:
   force  = [F_x, F_y, F_z]
   moment = [τ_roll, τ_pitch, τ_yaw]
</code></pre>
</section>

<section>
  <h3><code>src/palletrone_controller/src/allocator_controller.cpp</code></h3>
<img src="https://github.com/user-attachments/assets/5d45fe7c-532a-44e5-861f-2f94e959c40c" alt="Image" width="1000">
  <p>Sequential allocation scheme. 
Assumption: BLDC thrust response is faster than servo tilt response. 
First solve allocation with current fixed tilt <code>θ=C2_mea</code>, then update tilt angles from the resulting thrusts. </p>

  <h4>1) Allocation at fixed tilt (θ frozen)</h4>
  <p>Control objective: map desired rotational part and vertical force to per-motor thrusts at current tilt.</p>
  <pre><code>Eigen::Vector4d B1(Wrench(0), Wrench(1), tauz_r_sat, Wrench(5)); // [τ_roll, τ_pitch, τ_yaw_rotor, F_z]
Eigen::Matrix4d A1 = calc_A1(C2_mea_);
{
  Eigen::FullPivLU&lt;Eigen::Matrix4d&gt; lu_1(A1);
  if (lu_1.isInvertible()) C1_ = lu_1.solve(B1);
  else C1_ = (A1.transpose()*A1 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A1.transpose()*B1);
}</code></pre>
  <p>This is the control allocation for a tilted multirotor under fixed <code>θ</code>. 
If <code>A1</code> is not invertible, it falls back to a damped normal-equation solve.</p>

  <h4>2) Tilt-angle update for full actuation</h4>
  <pre><code>Eigen::Vector4d B2(Wrench(3), Wrench(4), tauz_t, 0.0); // [F_x, F_y, τ_yaw_tilt, 0]
Eigen::Matrix4d A2 = calc_A2(C1_, C2_mea_);
{
  Eigen::FullPivLU&lt;Eigen::Matrix4d&gt; lu_2(A2);
  if (lu_2.isInvertible()) C2_des_ = lu_2.solve(B2);
  else C2_des_ = (A2.transpose()*A2 + 1e-8*Eigen::Matrix4d::Identity()).ldlt().solve(A2.transpose()*B2);
}</code></pre>
  <p>This updates the tilt angles for full actuation. 
If <code>A2</code> is not invertible, it uses the same damped least-squares fallback.</p>

  <h4>3) Yaw trimming</h4>
  <img src="https://github.com/user-attachments/assets/0b828be1-ee13-4eea-876a-a3f75a4dceea" alt="diagram" width="500">
  <p>Low-pass estimate of the yaw reaction torque bias and residual saturation.</p>
  <pre><code>tauz_bar_ = lpf_alpha * Wrench(2) + (1.0 - lpf_alpha) * tauz_bar_;
double tauz_r     = Wrench(2) - tauz_bar_;
double tauz_r_sat = std::clamp(tauz_r, -2.0, 2.0);
double tauz_t     = tauz_bar_ + (tauz_r - tauz_r_sat);</code></pre>


  <h4>BLDC mapping</h4>
  <p>Convert allocated thrusts to motor speeds with <code>F = speed²</code> and yaw reaction <code>τ<sub>z</sub> = ζ·speed²</code> (coefficient 1 for thrust term).</p>
  <pre><code>double motor_speed[4];
motor_speed[0] = std::sqrt(std::max(0.0, C1_(0)/zeta));
motor_speed[1] = std::sqrt(std::max(0.0, C1_(1)/zeta));
motor_speed[2] = std::sqrt(std::max(0.0, C1_(2)/zeta));
motor_speed[3] = std::sqrt(std::max(0.0, C1_(3)/zeta));</code></pre>
  <p>Actuator vector ordering: <code>u = [f1,f2,f3,f4, th1,th2,th3,th4]</code>.</p>
</section>


<h2><code>plant/plant/plant.py</code></h2>

<section>
  <h4>1) Model</h4>
  <p>
    The MuJoCo model is provided in <code>plant/xml/Palletrone.xml</code>, and the scene in <code>plant/xml/scene.xml</code>.
    The mass and moments of inertia (MoI) are set to approximate real values. The scene config sets the background and a
    reference path.
  </p>
  <p><b>Note:</b> To change the reference trajectory, edit the scene accordingly.
     If you need a clean scene, remove <code>lines 21–53</code> in <code>scene.xml</code>.
  </p>
</section>

<section>
  <h4>2) Sensor noise implementation</h4>
  <p>
    Noisy IMU/state readings are generated by adding Gaussian noise. Parameters in
    <code>src/plant/plant/plant.py</code>:
  </p>
  <pre><code>SIG_POS  = 1e-3   # position noise
SIG_VEL  = 1e-3   # velocity noise
SIG_GYRO = 1e-3   # gyro noise
SIG_SERVO= 1e-4   # servo sensor noise
</code></pre>
  <p>Noise function:</p>
  <pre><code>def _noisy(self, x, s):
    return x + np.random.normal(0.0, s, size=x.shape)
</code></pre>
  <p>This adds zero-mean Gaussian noise with standard deviation <code>s</code> to the signal <code>x</code>.</p>
</section>

<section>
  <h4>3) Time delay implementation</h4>
  <p>
    A simple input time delay is applied to the received control vector. Parameter:
    <code>DELAY_TIME = 0.01</code> in <code>src/plant/plant/plant.py</code>.
    Only time delay is modeled; no shape deformation is applied.
  </p>
  <p>Core logic:</p>
  <pre><code>def _delay_step(self):
    i = self._delay_idx
    self._delay_buf[i] = self.ctrl_recv
    self._delay_idx = (i + 1) % self._delay_len
    return self._delay_buf[self._delay_idx]
</code></pre>
</section>



<h2>Reference</h2>

<section>
  <ul>
    <li><a href="https://doi.org/10.1109/LRA.2024.3416794" target="_blank" rel="noopener">
      The Palletrone Cart: Human-Robot Interaction-Based Aerial Cargo Transportation (IEEE RA-L 2024)</a></li>
    <li><a href="https://ieeexplore.ieee.org/document/11016760" target="_blank" rel="noopener">
      Aerial Dockable Multirotor UAVs: Design, Control, and Flight Time Extension Through In-Flight Battery Replacement (IEEE RA-L 2024)</a></li>
  </ul>
</section>

</body>
</html>
