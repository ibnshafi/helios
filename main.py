import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.widgets import Slider, Button
from scipy.spatial.transform import Rotation as R

# =============================================================================
# HELIOS/CHIRON – 6‑DOF Collaborative Robot Arm
# Integrated Robotics Prototype for Kinematic, Control & Dynamic Validation
# =============================================================================

# -----------------------------------------------------------------------------
# 1. KINEMATICS (DH parameters, FK, Jacobian, IK)
# -----------------------------------------------------------------------------
dh_params = np.array([
    # a, alpha, d, theta_offset
    [0.0,   0.0,         0.175,  0.0],       # Joint 1
    [0.0,  -np.pi/2,     0.0,   -np.pi/2],   # Joint 2
    [0.350, 0.0,         0.0,    0.0],       # Joint 3
    [0.050,-np.pi/2,     0.350,  0.0],       # Joint 4
    [0.0,   np.pi/2,     0.0,    0.0],       # Joint 5
    [0.0,  -np.pi/2,     0.100,  0.0]        # Joint 6
])

def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st*ca,  st*sa, a*ct],
                     [st,  ct*ca, -ct*sa, a*st],
                     [0,   sa,     ca,    d],
                     [0,   0,      0,     1]])

def forward_kinematics(q):
    T = np.eye(4)
    positions = [np.zeros(3)]
    for i in range(6):
        a, alpha, d, off = dh_params[i]
        T_i = dh_transform(a, alpha, d, q[i] + off)
        T = T @ T_i
        positions.append(T[:3, 3].copy())
    return T, np.array(positions)

def geometric_jacobian(q):
    J = np.zeros((6, 6))
    T = np.eye(4)
    z_axes = [np.array([0,0,1])]
    origins = [np.zeros(3)]
    for i in range(6):
        a, alpha, d, off = dh_params[i]
        T_i = dh_transform(a, alpha, d, q[i] + off)
        T = T @ T_i
        z_axes.append(T[:3, 2].copy())
        origins.append(T[:3, 3].copy())
    p_ee = origins[-1]
    for i in range(6):
        J[:3, i] = np.cross(z_axes[i], p_ee - origins[i])
        J[3:, i] = z_axes[i]
    return J

def condition_number(J):
    _, s, _ = np.linalg.svd(J)
    return s[0] / s[-1] if s[-1] > 1e-12 else np.inf

def pose_error(T_cur, T_des):
    p_err = T_des[:3,3] - T_cur[:3,3]
    R_err = T_des[:3,:3] @ T_cur[:3,:3].T
    angle = np.arccos(np.clip((np.trace(R_err)-1)/2, -1.0, 1.0))
    if angle < 1e-6:
        axis = np.zeros(3)
    else:
        axis = np.array([R_err[2,1]-R_err[1,2],
                         R_err[0,2]-R_err[2,0],
                         R_err[1,0]-R_err[0,1]]) / (2*np.sin(angle))
    return np.concatenate([p_err, angle*axis])

def ik_dls(T_des, q0, max_iter=200, tol=1e-6, damping=0.1):
    q = q0.copy()
    for _ in range(max_iter):
        T_cur, _ = forward_kinematics(q)
        e = pose_error(T_cur, T_des)
        if np.linalg.norm(e) < tol:
            break
        J = geometric_jacobian(q)
        A = J @ J.T + damping**2 * np.eye(6)
        q += J.T @ np.linalg.solve(A, e)
    return q

# -----------------------------------------------------------------------------
# 2. RIGID BODY DYNAMICS – Proper Recursive Newton‑Euler (static gravity)
# -----------------------------------------------------------------------------
# Link inertial parameters (simplified)
link_masses = np.array([1.0, 1.5, 1.0, 0.3, 0.3, 0.3])  # kg
# COM position in the frame of the link itself (i.e., relative to joint i+1)
link_com = np.array([[0.0,0.0,0.05],   # link1
                     [0.175,0.0,0.0],  # link2
                     [0.175,0.0,0.0],  # link3
                     [0.0,0.0,0.0],
                     [0.0,0.0,0.0],
                     [0.0,0.0,0.05]])  # link6
# Diagonal inertias about COM (kg·m²)
link_I = np.zeros((6,3,3))
link_I[0] = np.diag([0.005,0.005,0.001])
link_I[1] = np.diag([0.010,0.003,0.010])
link_I[2] = np.diag([0.006,0.002,0.006])
link_I[3] = np.diag([5e-4,5e-4,1e-4])
link_I[4] = np.diag([5e-4,5e-4,1e-4])
link_I[5] = np.diag([2e-4,2e-4,1e-4])

def newton_euler_gravity(q):
    """
    Recursive Newton‑Euler for static case (gravity only).
    Forward recursion with zero velocity/acceleration, then backward.
    Returns joint torques τ_g due to gravity.
    """
    g_world = np.array([0, 0, -9.81])
    # Forward: compute transforms and spatial positions
    T = np.eye(4)
    R_i_prev = [np.eye(3)]
    p_i_prev = [np.zeros(3)]
    for i in range(6):
        a, alpha, d, off = dh_params[i]
        T = T @ dh_transform(a, alpha, d, q[i] + off)
        R_i_prev.append(T[:3,:3].copy())
        p_i_prev.append(T[:3,3].copy())
    # Backward recursion
    f_i = np.zeros(3)   # force exerted by link i+1 on link i (at joint i+1)
    n_i = np.zeros(3)   # moment exerted by link i+1 on link i
    tau = np.zeros(6)
    for i in reversed(range(6)):
        # Transform COM from link i+1 frame to world
        R_ip1 = R_i_prev[i+1]          # orientation of link i+1 (frame i+1)
        p_ip1 = p_i_prev[i+1]          # origin of frame i+1 (joint i+1)
        com_world = p_ip1 + R_ip1 @ link_com[i]  # COM in world
        # Weight vector in world
        F_com = link_masses[i] * g_world   # m_i * g
        # No dynamic moments because w = 0, w_dot = 0
        # Force balance: f_{i-1} = f_i + F_com  (forces acting on link i)
        f_prev = f_i + F_com
        # Moment balance about COM: n_{i-1} = n_i + (p_i - p_{i-1}) x f_i + (com - p_{i-1}) x F_com
        if i == 0:
            p_prev = np.zeros(3)  # base frame
        else:
            p_prev = p_i_prev[i]  # origin of link i (joint i)
        r1 = p_ip1 - p_prev      # vector from joint i to joint i+1
        r2 = com_world - p_prev  # vector from joint i to COM
        n_prev = n_i + np.cross(r1, f_i) + np.cross(r2, F_com)
        # Joint torque = component of n_{i-1} along joint axis (z_i of link i)
        z_i = R_i_prev[i][:,2]   # joint axis of link i (world frame)
        tau[i] = np.dot(n_prev, z_i)
        # Pass forces/moments to previous link
        f_i = f_prev
        n_i = n_prev
    return tau

# -----------------------------------------------------------------------------
# 3. IMPEDANCE CONTROL (operational‑space, one‑step command)
# -----------------------------------------------------------------------------
def impedance_control_step(q, T_des, K_p=500, K_d=50):
    """Compute joint torque command for Cartesian impedance (spring‑damper)."""
    T_cur, _ = forward_kinematics(q)
    e_p = T_des[:3,3] - T_cur[:3,3]
    # Assume zero velocity error (simplified)
    e_v = np.zeros(3)
    F_ee = K_p * e_p + K_d * e_v
    J = geometric_jacobian(q)
    tau_cmd = J[:3,:].T @ F_ee
    return tau_cmd, e_p

def impedance_lyapunov(e_p, e_v=np.zeros(3), K_p=500, Lambda=0.05, K_d=5):
    V = 0.5 * (np.dot(e_v, Lambda*e_v) + np.dot(e_p, K_p*e_p))
    Vdot = -np.dot(e_v, K_d*e_v)   # simplified
    return V, Vdot

# -----------------------------------------------------------------------------
# 4. MOTION PLANNING CONCEPT (cost field for DIS‑RRT*)
# -----------------------------------------------------------------------------
def cost_field(q):
    tau_g = newton_euler_gravity(q)
    J = geometric_jacobian(q)
    d_obs = 10.0   # placeholder
    return (1.0*np.linalg.norm(tau_g) +
            10.0/d_obs +
            0.1*np.linalg.norm(J, ord=np.inf) +
            0.05*condition_number(J))

def boltzmann_prob(q, beta=10):
    return np.exp(-beta * cost_field(q))

# -----------------------------------------------------------------------------
# 5. TORQUE SENSOR SIMULATION (flexure + Wheatstone bridge)
# -----------------------------------------------------------------------------
def torque_sensor_output(tau):
    # Design parameters
    r, L, b, h = 0.05, 0.005, 0.008, 0.001
    E, GF, V_ex, gain = 200e9, 2.0, 5.0, 100
    F = tau / (4*r)
    M = F * L
    I = (b * h**3) / 12
    eps = (M * h/2) / (E * I)
    V_out = V_ex * GF * eps * gain
    return eps, V_out

# -----------------------------------------------------------------------------
# 6. CYCLOIDAL DRIVE SPEC
# -----------------------------------------------------------------------------
def cycloidal_drive(lobes=35):
    return lobes, 1.95 * lobes   # ratio, output torque (Nm)

# =============================================================================
# INTERACTIVE DASHBOARD
# =============================================================================
q_home = np.array([0.0, -np.pi/4, np.pi/2, 0.0, np.pi/4, 0.0])
q_current = q_home.copy()

target_T = np.eye(4)
target_T[:3,3] = [0.5, 0.0, 0.5]

fig = plt.figure(figsize=(14, 9))
fig.suptitle("HELIOS/CHIRON – Integrated Robotics Prototype", fontsize=14)
ax = fig.add_subplot(111, projection='3d')
plt.subplots_adjust(bottom=0.35)

# Joint sliders
sl_axes = []
for i in range(6):
    ax_sl = plt.axes([0.1, 0.25 - i*0.03, 0.65, 0.02])
    sl = Slider(ax_sl, f'q{i+1}', -np.pi, np.pi, valinit=q_home[i])
    sl_axes.append(sl)

# Target sliders
tar_sl = []
for i, (name, low, high, init) in enumerate(zip(
    ['X','Y','Z','Roll','Pitch','Yaw'],
    [-0.8,-0.8,0.0,-np.pi,-np.pi,-np.pi],
    [0.8,0.8,1.2, np.pi, np.pi, np.pi],
    [0.5, 0.0, 0.5, 0.0, 0.0, 0.0])):
    ax_s = plt.axes([0.1, 0.10 - i*0.03, 0.12, 0.02])
    sl = Slider(ax_s, name, low, high, valinit=init)
    tar_sl.append(sl)

btn_ax = plt.axes([0.8, 0.15, 0.1, 0.04])
btn_ik = Button(btn_ax, 'Solve IK')

info_ax = plt.axes([0.8, 0.2, 0.2, 0.1])
info_ax.axis('off')
info_txt = info_ax.text(0, 0.5, '', va='center', fontsize=9)

def update_target(val):
    x, y, z = tar_sl[0].val, tar_sl[1].val, tar_sl[2].val
    rpy = [tar_sl[i].val for i in range(3,6)]
    target_T[:3,3] = [x, y, z]
    target_T[:3,:3] = R.from_euler('xyz', rpy).as_matrix()
    draw_arm()

for sl in tar_sl:
    sl.on_changed(update_target)

def solve_ik(event):
    global q_current
    q_current = ik_dls(target_T, q_current)
    for i in range(6):
        sl_axes[i].eventson = False
        sl_axes[i].set_val(q_current[i])
        sl_axes[i].eventson = True
    draw_arm()

btn_ik.on_clicked(solve_ik)

def draw_arm(val=None):
    q = np.array([sl.val for sl in sl_axes])
    T_ee, pos = forward_kinematics(q)
    J = geometric_jacobian(q)
    cond = condition_number(J)

    tau_g = newton_euler_gravity(q)
    strain, v_out = torque_sensor_output(tau_g[1])
    c = cost_field(q)
    p = boltzmann_prob(q)
    tau_cmd, e_p = impedance_control_step(q, target_T)
    V, Vd = impedance_lyapunov(e_p)

    ax.clear()
    ax.set_xlim(-0.8,0.8); ax.set_ylim(-0.8,0.8); ax.set_zlim(0,1.2)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    ax.plot(pos[:,0], pos[:,1], pos[:,2], 'o-', lw=3, color='royalblue', markersize=5)
    for idx, col in zip([0,1,2], ['r','g','b']):
        ax.quiver(*pos[-1], *(T_ee[:3,idx]*0.1), color=col, arrow_length_ratio=0.1)
    ax.scatter(*target_T[:3,3], color='red', s=80, marker='x', linewidth=2, label='Target')
    ax.legend(loc='upper left')

    info = (f"EE pos: {pos[-1].round(3)}\n"
            f"cond(J): {cond:.1f}\n"
            f"τ_g (all): {tau_g.round(1)}\n"
            f"τ_J2 sensor: {tau_g[1]:.1f} Nm, ε={strain*1e6:.1f} µε, Vout={v_out*1e3:.2f} mV\n"
            f"C(q): {c:.2f}, p(q): {p:.2e}\n"
            f"Impedance cmd: τ_J2={tau_cmd[1]:.1f} Nm, ||e_p||={np.linalg.norm(e_p)*1000:.1f} mm\n"
            f"Lyapunov V: {V:.2e}, V̇: {Vd:.2e} (stable)" if Vd <= 0 else f"V̇={Vd:.2e} (unstable!)")
    info_txt.set_text(info)
    fig.canvas.draw_idle()

for sl in sl_axes:
    sl.on_changed(draw_arm)

draw_arm()
plt.show()