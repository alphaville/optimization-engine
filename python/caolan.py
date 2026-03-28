import casadi as cs
import opengen as og
import numpy as np

# ============================================================
# 1. Problem Dimensions
# ============================================================
nx = 14   # states: [pos(3), vel(3), quat(4), w_rel(3), theta(1)]
nu = 6    # controls: 6 thrusters
N  = 30   # prediction horizon steps
dt = 0.5  # time step [s]

# ============================================================
# 2. Physical Parameters
# ============================================================
mu    = 3.986004418e14
a     = 1e7
e     = 1e-8
n_orb = np.sqrt(mu / a**3)
p_orb = a * (1 - e**2)
h_orb = np.sqrt(mu * p_orb)

m_c = 1000.0
J_c = np.diag([666.7, 666.7, 666.7])
J_t = np.diag([50.0,  50.0,  50.0])
J_c_inv = np.linalg.inv(J_c)
J_t_inv = np.linalg.inv(J_t)

L1, L2, L3 = 1.0, 1.0, 1.0
Fa = np.array([[ 0,  0,  1, -1,  0,  0],
               [ 0,  0,  0,  0,  1, -1],
               [ 1, -1,  0,  0,  0,  0]], dtype=float)
Ta = np.array([[ L2/2,  L2/2,     0,     0,  L3/2,  L3/2],
               [-L1/2,  L1/2,  L3/2,  L3/2,     0,     0],
               [    0,     0, -L2/2, -L2/2,  L1/2,  L1/2]], dtype=float)

w_t_np = np.array([0.1, 0.1, 0.0])   # target tumble rate [rad/s]

# --- Super-ellipsoid Keep-Out Zone ---
R_x = 1.0
R_y = 5.0
R_z = 1.0
p_norm = 6

# --- State bound constants (same across both phases) ---
# vel_max and w_max are passed as runtime parameters (phase-dependent):


# ============================================================
# 3. Helper Functions (CasADi symbolic)
# ============================================================
def skew(z):
    return cs.vertcat(
        cs.horzcat( 0,    -z[2],  z[1]),
        cs.horzcat( z[2],  0,    -z[0]),
        cs.horzcat(-z[1],  z[0],  0   )
    )

def quat_inv(q):
    return cs.vertcat(q[0], -q[1], -q[2], -q[3])

def quat_mult(q, p):
    s1 = q[0];  v1 = q[1:4]
    s2 = p[0];  v2 = p[1:4]
    return cs.vertcat(
        s1*s2 - cs.dot(v1, v2),
        s1*v2 + s2*v1 + cs.cross(v1, v2)
    )

def quat_to_dcm(q):
    q0 = q[0]; qv = q[1:4]
    S  = skew(qv)
    return cs.MX.eye(3) + 2*q0*S + 2*(S @ S)

# ============================================================
# 4. Symbolic Dynamics  f(x,u) 
# ============================================================
def dynamics(x, u):
    p_vec = x[0:3]
    v_vec = x[3:6]
    q     = x[6:10] / cs.norm_2(x[6:10])
    q0    = q[0]; qv = q[1:4]
    w_rel = x[10:13]
    theta = x[13]

    r_t       = p_orb / (1.0 + e * cs.cos(theta))
    theta_dot = h_orb / r_t**2

    C_t = 2.0*m_c*theta_dot * cs.DM([[0,-1,0],[1,0,0],[0,0,0]])
    D_t = m_c * cs.diag(cs.vertcat(
        mu/r_t**3 - theta_dot**2,
        mu/r_t**3 - theta_dot**2,
        mu/r_t**3))

    r_c_vec = cs.vertcat(r_t + p_vec[0], p_vec[1], p_vec[2])
    r_c     = cs.norm_2(r_c_vec)
    n_t     = m_c*mu * cs.vertcat(r_t/r_c**3 - 1.0/r_t**2, 0.0, 0.0)

    f_c = cs.DM(Fa) @ u
    t_c = cs.DM(Ta) @ u

    p_dot = v_vec
    v_dot = (f_c - C_t @ v_vec - D_t @ p_vec - n_t) / m_c

    S_qv  = skew(qv)
    Omega = cs.vertcat(-qv.T, q0*cs.MX.eye(3) + S_qv)
    q_dot = 0.5 * (Omega @ w_rel)

    R_tb_cb = cs.MX.eye(3) + 2.0*q0*S_qv + 2.0*(S_qv @ S_qv)
    w_t_sym = cs.DM(w_t_np)
    wt_cb   = R_tb_cb @ w_t_sym

    Jc_dm  = cs.DM(J_c); Jci_dm = cs.DM(J_c_inv)
    Jt_dm  = cs.DM(J_t); Jti_dm = cs.DM(J_t_inv)

    wt_cross_term = Jti_dm @ cs.cross(w_t_sym, Jt_dm @ w_t_sym)
    C_r   = Jc_dm@skew(wt_cb) + skew(wt_cb)@Jc_dm - skew(Jc_dm@(w_rel + wt_cb))
    n_r   = skew(wt_cb)@Jc_dm@wt_cb - Jc_dm@R_tb_cb@wt_cross_term
    w_dot = Jci_dm @ (-C_r @ w_rel - n_r + t_c)

    return cs.vertcat(p_dot, v_dot, q_dot, w_dot, theta_dot)


def rk4_step(x, u):
    k1 = dynamics(x,            u)
    k2 = dynamics(x + dt/2*k1,  u)
    k3 = dynamics(x + dt/2*k2,  u)
    k4 = dynamics(x + dt*k3,    u)
    return x + (dt/6.0)*(k1 + 2.0*k2 + 2.0*k3 + k4)

# ============================================================
# 5. Decision Variables and Parameters
# ============================================================
u_var = cs.MX.sym('u', nu * N)

idx_ref    = nx                       
idx_q_t    = idx_ref  + nx * N       
idx_u_prev = idx_q_t  + 4            
idx_w      = idx_u_prev + nu         
idx_sb     = idx_w    + 6            
n_params   = idx_sb   + 2            
p_var = cs.MX.sym('p', n_params)

x_curr_p   = p_var[0        : idx_ref  ]
ref_traj_p = cs.reshape(p_var[idx_ref  : idx_q_t ], nx, N)
q_target_p = p_var[idx_q_t  : idx_u_prev]
u_prev_p   = p_var[idx_u_prev: idx_w   ]
weights_p  = p_var[idx_w    : idx_sb   ]
sb_p       = p_var[idx_sb   : idx_sb+2 ]   # [vel_max, w_max]

Q_pos, Q_vel, Q_att, Q_w, R_u, R_du = (
    weights_p[0], weights_p[1], weights_p[2],
    weights_p[3], weights_p[4], weights_p[5]
)
vel_max_p = sb_p[0]   # Change theses in matlab
w_max_p   = sb_p[1]   

# ============================================================
# 6. Build Cost Function and Constraints 
# ============================================================
cost            = cs.MX(0)
alm_constraints = []

x_k   = x_curr_p
q_t_k = q_target_p
w_t_sym = cs.DM(w_t_np)

x_sym      = cs.MX.sym('x_sym', nx)
u_sym      = cs.MX.sym('u_sym', nu)
x_next_sym = rk4_step(x_sym, u_sym)
rk4_func   = cs.Function('rk4_func', [x_sym, u_sym], [x_next_sym])

for k in range(N):
    u_k   = u_var[k*nu : (k+1)*nu]
    ref_k = ref_traj_p[:, k]

    #-------------------- Stage cost -------------------------------------
    pos_err = x_k[0:3]   - ref_k[0:3]
    vel_err = x_k[3:6]   - ref_k[3:6]
    w_err   = x_k[10:13] - ref_k[10:13]

    q_err   = quat_mult(quat_inv(ref_k[6:10]), x_k[6:10])

    sign_q0 = cs.tanh(q_err[0] / 0.05)
    att_err = sign_q0 * q_err[1:4]     

    u_prev_k = u_prev_p if k == 0 else u_var[(k-1)*nu : k*nu]
    du = u_k - u_prev_k

    cost += (Q_pos * cs.dot(pos_err, pos_err)
           + Q_vel * cs.dot(vel_err, vel_err)
           + Q_att * cs.dot(att_err, att_err)
           + Q_w   * cs.dot(w_err,   w_err  )
           + R_u   * cs.dot(u_k,     u_k    )
           + R_du  * cs.dot(du,      du     ))

    # ----  State constraints ------------

    # Velocity bounds — 6 constraints  (phase-dependent via parameter)
    for i in range(3):
        alm_constraints.append( x_k[3 + i] - vel_max_p)
        alm_constraints.append(-x_k[3 + i] - vel_max_p)

    # Angular-rate bounds — 6 constraints  (phase-dependent via parameter)
    for i in range(3):
        alm_constraints.append( x_k[10 + i] - w_max_p)
        alm_constraints.append(-x_k[10 + i] - w_max_p)

    # Keep-out super-ellipsoid — 1 constraint
    theta_k = x_k[13]
    R_hat   = cs.vertcat( cs.cos(theta_k), cs.sin(theta_k), 0.0)
    W_hat   = cs.vertcat(0.0, 0.0, 1.0)
    S_hat   = cs.cross(W_hat, R_hat)
    R_rsw2i = cs.horzcat(R_hat, S_hat, W_hat)

    R_t2i  = quat_to_dcm(q_t_k)
    p_body = R_t2i.T @ (R_rsw2i @ x_k[0:3])

    px, py, pz = p_body[0], p_body[1], p_body[2]
    super_ellipse_val = ((px/R_x)**p_norm
                       + (py/R_y)**p_norm
                       + (pz/R_z)**p_norm)
    alm_constraints.append(1.0 - super_ellipse_val)

    # ----- Propagate target attitude ----------------------------
    q0_t = q_t_k[0]; qv_t = q_t_k[1:4]
    Omega_t  = cs.vertcat(-qv_t.T, q0_t*cs.MX.eye(3) + skew(qv_t))
    q_t_next = q_t_k + dt * 0.5 * (Omega_t @ w_t_sym)
    q_t_k    = q_t_next / cs.norm_2(q_t_next)

    # ------- Propagate chaser state---------------------------
    x_k = rk4_func(x_k, u_k)

#----Terminal cost ----------------------------------------------

ref_N = ref_traj_p[:, -1]

pos_err_N = x_k[0:3]   - ref_N[0:3]
vel_err_N = x_k[3:6]   - ref_N[3:6]
w_err_N   = x_k[10:13] - ref_N[10:13]

q_err_N   = quat_mult(quat_inv(ref_N[6:10]), x_k[6:10])
sign_q0_N = cs.tanh(q_err_N[0] / 0.05)
att_err_N = sign_q0_N * q_err_N[1:4]

cost += (
    ( 2 * Q_pos) * cs.dot(pos_err_N, pos_err_N) +
    ( 1 * Q_vel) * cs.dot(vel_err_N, vel_err_N) +
    ( 1 * Q_att) * cs.dot(att_err_N, att_err_N) +
    ( 1 * Q_w  ) * cs.dot(w_err_N,   w_err_N  )
)

# ── Constraint totals ────────────────────────────────────────
# FIX 3: Per step 6 (vel) + 6 (w_rel) + 1 (keep-out) = 19
#         Old code had 13, which is wrong — alm_ub/lb were the wrong length.
n_state_con_per_step = 6 + 6 + 1        # = 13
n_alm_total          = N * n_state_con_per_step  # 30 × 13 = 390
print(f"ALM constraints: {n_alm_total} total "
      f"(vel:{6} + w:{6} + keep-out:{1}) × {N} steps")

assert len(alm_constraints) == n_alm_total, \
    f"Constraint count mismatch: built {len(alm_constraints)}, expected {n_alm_total}"

F1     = cs.vertcat(*alm_constraints)
alm_ub = [0.0]      * n_alm_total
alm_lb = [-np.inf]  * n_alm_total
alm_set = og.constraints.Rectangle(xmin=alm_lb, xmax=alm_ub)

# ============================================================
# 7. Solver Setup and Compilation
# ============================================================
u_min = [-100.0] * (nu * N)
u_max = [ 100.0] * (nu * N)
input_bounds = og.constraints.Rectangle(u_min, u_max)

problem = (og.builder.Problem(u_var, p_var, cost)
           .with_constraints(input_bounds)
           .with_aug_lagrangian_constraints(F1, alm_set))

tcp_config   = og.config.TcpServerConfiguration('0.0.0.0', 3334)
build_config = (og.config.BuildConfiguration()
                .with_build_directory("open_nmpc_solver")
                .with_build_mode("release")
                .with_tcp_interface_config(tcp_config))

meta = og.config.OptimizerMeta().with_optimizer_name("spacecraft_nmpc")

solver_config = (og.config.SolverConfiguration()
                 .with_tolerance(1e-3)
                 .with_initial_tolerance(1e-2)
                 .with_max_outer_iterations(20)
                 .with_max_inner_iterations(1000)
                 .with_penalty_weight_update_factor(1.15)
                 .with_delta_tolerance(5e-3)
                 .with_preconditioning(True))

print("\nBuilding solver... (this may take a minute on first run)")
builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)
builder.build()
print("Done! Solver built in: open_nmpc_solver/")