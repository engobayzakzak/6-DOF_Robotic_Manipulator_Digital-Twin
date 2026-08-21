# Technical Report: Modeling, Differential Kinematics, and Real-Time Control of a 6-DOF Industrial Digital-Twin

---

## 1. Mathematical Modeling & Kinematic Chain

The simulated robot is a 6-DOF serial anthropomorphic manipulator with a spherical wrist. The kinematic chain consists of six revolute joints ($q_1 \dots q_6$).

### 1.1 Standard Denavit-Hartenberg (DH) Parametrization
The coordinate transformations between successive reference frames $i-1$ and $i$ are governed by:

$$T_i^{i-1} = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\sin\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

#### Manipulator DH Parameters Table

| Link $i$ | $\theta_i\text{ [rad]}$ | $d_i\text{ [m]}$ | $a_i\text{ [m]}$ | $\alpha_i\text{ [rad]}$ | Description |
| :---: | :---: | :---: | :---: | :---: | :--- |
| **1** | $q_1$ | $0.10$ | $0.00$ | $+\pi/2$ | Base Yaw Rotation |
| **2** | $q_2$ | $0.08$ | $0.40$ | $0.00$ | Shoulder Pitch |
| **3** | $q_3$ | $-0.06$ | $0.35$ | $0.00$ | Elbow Pitch |
| **4** | $q_4$ | $0.35$ | $0.00$ | $+\pi/2$ | Wrist Roll |
| **5** | $q_5$ | $0.08$ | $0.00$ | $-\pi/2$ | Wrist Pitch |
| **6** | $q_6$ | $0.08$ | $0.00$ | $0.00$ | Wrist Yaw / Tool Flange |

### 1.2 Forward Kinematics Product
The global end-effector pose $\mathbf{T}_6^0(q) \in SE(3)$ relative to the inertial base frame is computed by:

$$\mathbf{T}_6^0(q) = \prod_{i=1}^6 T_i^{i-1}(q_i) = \begin{bmatrix} \mathbf{R}(q) & \mathbf{p}(q) \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$

---

## 2. Differential Kinematics & Singularity-Damped Teleoperation

### 2.1 Geometric Jacobian Derivation
The relationship mapping joint velocity space $\dot{q} \in \mathbb{R}^6$ to task-space twist $\dot{\mathbf{x}} = \begin{bmatrix} \mathbf{v}^T & \boldsymbol{\omega}^T \end{bmatrix}^T \in \mathbb{R}^6$ is defined by the Geometric Jacobian $\mathbf{J}(q) \in \mathbb{R}^{6 \times 6}$:

$$\dot{\mathbf{x}} = \mathbf{J}(q)\dot{q} = \begin{bmatrix} \mathbf{J}_{v1} & \dots & \mathbf{J}_{v6} \\ \mathbf{J}_{\omega 1} & \dots & \mathbf{J}_{\omega 6} \end{bmatrix} \dot{q}$$

For revolute joints:
$$\mathbf{J}_{vi} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{p}_{i-1}), \quad \mathbf{J}_{\omega i} = \mathbf{z}_{i-1}$$

### 2.2 Singularity Robust Damped Least-Squares (DLS)
Near kinematic singular configurations where $\det(\mathbf{J}(q)) \to 0$ (such as wrist alignment or full arm extension), standard pseudo-inverse $\mathbf{J}^\dagger = \mathbf{J}^T(\mathbf{J}\mathbf{J}^T)^{-1}$ produces unbounded joint velocity spikes. 

We formulate a Levenberg-Marquardt Damped Inverse Kinematics solver with dynamic damping coefficient $\lambda$:

$$\mathbf{J}^* = \mathbf{J}^T \left( \mathbf{J}\mathbf{J}^T + \lambda^2 \mathbf{I}_{6 \times 6} \right)^{-1}$$

$$\lambda^2 = \begin{cases} 
0, & w \ge w_0 \\
\lambda_{\max}^2 \left(1 - \frac{w}{w_0}\right)^2, & w < w_0 
\end{cases}$$

where $w = \sqrt{\det(\mathbf{J}\mathbf{J}^T)}$ represents Yoshikawa's Manipulability Index. When $w < w_0$, damping is automatically injected, gracefully trading minimal Cartesian tracking error for numerical stability and hardware safety.

---

## 3. High-Level Trajectory Optimization & Sampling-Based Motion Planning

Trajectory synthesis was evaluated across three distinct OMPL algorithmic classes:

1. **RRT-Connect (Greedy Sampling):** Grows two rapid exploration trees from start $q_{\text{start}}$ and goal $q_{\text{goal}}$. Achieves lowest compute latency ($8.42\text{ ms}$), making it ideal for real-time collision reaction.
2. **RRT\* (Asymptotically Optimal):** Employs neighbor rewiring within radius $r(\text{card}(V))$ to minimize total joint path integral $\int \|\dot{q}\|_2 dt$. Generates the shortest trajectories ($2.891\text{ rad}$) at the expense of higher planning overhead ($284.15\text{ ms}$).
3. **PRM\* (Probabilistic Roadmap):** Constructs a multi-query collision-free graph representation across the entire configuration space $\mathcal{C}_{\text{free}}$.

---

## 4. Software-in-the-Loop Control Architecture

                  [ Gazebo Sim (1000 Hz) ]
                             │
                  /clock (Simulation Time)
                             ▼
                 ┌───────────────────────┐
                 │    `ros2_control`     │
                 │  Hardware Abstraction │
                 └───────────┬───────────┘
                             │
            /joint_states (sensor_msgs/JointState)
                             ▼
                 ┌───────────────────────┐
                 │   `move_group` Node   │
                 │  - Planning Scene Mon │
                 │  - OMPL Path Planners │
                 └───────────┬───────────┘
                             │
       /arm_controller/follow_joint_trajectory (Action)
                             ▼
                 ┌───────────────────────┐
                 │ JointTrajectoryContr. │
                 │ Quintic Spline Interp │
                 └───────────────────────┘

The system operates across two synchronized real-time control domains:
1. **Low-Level Hardware Control ($1000\text{ Hz}$):** `gz_ros2_control` directly commands actuator position setpoints with torque limits.
2. **Planning & Teleoperation Domain ($50\text{ Hz} - 100\text{ Hz}$):** `MoveIt Servo` and `PickAndPlaceSequencer` stream dynamically parameter-checked splines.

---

## 5. Summary of Key Engineering Solutions

| Challenge Encountered | Root Cause | Engineering Resolution |
| :--- | :--- | :--- |
| **MoveGroup Plugin Crash** | Jazzy moved TOTG from request to response adapter. | Updated `ompl_planning.yaml` to declare `AddTimeOptimalParameterization` as a response adapter. |
| **MoveGroup Startup Error 99999** | Missing non-adjacent link pairs in SRDF ACM causing false-positive self-collision checks. | Constructed complete 21-pair Allowed Collision Matrix in `arm_6dof.srdf`. |
| **Trajectory Goal Rejection** | Wall-clock vs. simulation-clock timestamp skew between Python node and Gazebo. | Enforced `use_sim_time:=true` parameter synchronization across all nodes. |
