from wobl_sim.robot import Robot
from dm_control.locomotion.arenas import Floor
from dm_control import mjcf
import numpy as np
import control
import mujoco
import mediapy as media


def print_model_info(mjcf_model: mjcf.RootElement, model):
    print("=== Model Info ===")
    print("nq =", model.nq, " nv =", model.nv, " nu =", model.nu)
    print()

    # Joint names and their qpos/qvel addresses
    print("=== Joints ===")
    for i, jn in enumerate(mjcf_model.find_all("joint")):
        addr_qpos = model.jnt_qposadr[i] if i < len(model.jnt_qposadr) else None
        addr_qvel = model.jnt_dofadr[i] if i < len(model.jnt_dofadr) else None
        print(f"Joint {i}: {jn}  qpos[{addr_qpos}]  qvel[{addr_qvel}]")

    print()

    # Actuators
    print("=== Actuators ===")
    for i, an in enumerate(mjcf_model.find_all("actuator")):
        trnid = model.actuator_trnid[i]
        joint_id = trnid[0]
        print(f"Actuator {i}: {an} -> joint {joint_id}")


# Function to compute the full-body Center of Mass
def compute_total_com(model, data):
    total_mass = 0.0
    com = np.zeros(3)

    for i in range(model.nbody):
        body_mass = model.body_mass[i]
        # Get world position of this body's CoM
        body_com = data.xipos[i]
        print(body_com)
        com += body_mass * body_com
        total_mass += body_mass

    return com / total_mass if total_mass > 0 else com

def find_equilibrium(robot: Robot, physics: mjcf.Physics):
    com = compute_total_com(physics.model, physics.data)
    com_x_offset = com[0]
    com_z_offset = com[2] - robot.mjcf_model.find("joint", "L_foot").pos[2]
    theta_eq = -com_x_offset / com_z_offset
    print(f"Approx equilibrium pitch (rad): {theta_eq:.4f}")

    # Apply small-angle to root freejoint quaternion
    qpos = physics.data.qpos.copy()
    qpos[4] = theta_eq / 2  # qx component of quaternion
    physics.data.qpos[:] = qpos

def plot_stuff(K, A_reduced, B_reduced):
    import matplotlib.pyplot as plt

    # --- Inputs: your reduced matrices ---
    A = A_reduced       # 3x3 [theta, theta_dot, forward velocity]
    B = B_reduced       # 3x2 [L_wheel, R_wheel]
    K = K               # 2x3 LQR gain

    # --- Make left/right wheel gains identical for comparison (optional) ---
    K_sym = 0.5 * (K[0,:] + K[1,:])
    K = np.array([K_sym, K_sym])  # both wheels identical

    # Closed-loop system for state evolution
    A_cl = A - B @ K

    # Simulation parameters
    dt = 0.002  # 2 ms timestep
    T = 0.5     # 5 seconds
    N = int(T / dt)
    time = np.linspace(0, T, N)

    # Initial state: 1 deg forward lean
    x = np.zeros((3,N))
    x[:,0] = [np.deg2rad(1.0), 0, 0]  # [theta, theta_dot, forward velocity]

    # Arrays to store wheel commands
    u = np.zeros((2,N))

    # Simulate
    for i in range(1,N):
        
        u[:,i-1] = -K @ x[:,i-1]          # wheel velocity commands
        #u[:,i-1] = np.clip(u[:,i-1], -10, 10)  # limit wheel speeds
        dx = A @ x[:,i-1] + B @ u[:,i-1]  # continuous-time dynamics
        x[:,i] = x[:,i-1] + dx*dt          # Euler integration

    # Final wheel command at last step
    u[:, -1] = -K @ x[:, -1]

    # --- Plot results ---
    plt.figure(figsize=(10,5))
    plt.subplot(2,1,1)
    plt.plot(time, np.rad2deg(x[0,:]), label='Pitch θ [deg]')
    plt.plot(time, x[2,:], label='Forward velocity v [m/s]')
    plt.xlabel('Time [s]')
    plt.grid(True)
    plt.legend()
    plt.title('State evolution (linearized closed-loop)')

    plt.subplot(2,1,2)
    plt.plot(time, u[0,:], label='Left wheel command')
    plt.plot(time, u[1,:], label='Right wheel command', linestyle='--')
    plt.xlabel('Time [s]')
    plt.ylabel('Wheel velocity command [units]')
    plt.grid(True)
    plt.legend()
    plt.title('Wheel commands')

    plt.tight_layout()
    plt.show()


def main():
    robot = Robot()
    arena = Floor(reflectance=0.0)
    arena.add_free_entity(robot)

    physics = mjcf.Physics.from_mjcf_model(arena.mjcf_model)
    dt = physics.model.opt.timestep
    print("time stepp:", dt)

    #robot.set_pose(physics, np.array([0, 0, 0.237]), np.array([1, 0, 0, 0]))

    find_equilibrium(robot, physics)

    # --- 1) Find dimensions ---
    nq = physics.model.nq       # number of generalized coords
    nv = physics.model.nv       # number of velocities
    nu = physics.model.nu       # number of actuators
    nx = nv * 2   # state vector [qpos, qvel]

    print("nq =", nq, " nv =", nv, " nu =", nu)

    physics.forward()

    # states = [qpos; qvel], input = actuators
    # Allocate matrices for discrete-time linearization
    A = np.zeros((nx, nx))
    B = np.zeros((nx, nu))

    # mjd_transitionFD computes [A,B] around (qpos,qvel,u)
    # It expects data to be at the linearization point
    eps = 1e-6
    flg_centered = True
    mujoco.mjd_transitionFD(physics.model.ptr, physics.data.ptr, eps, flg_centered, A, B, None, None)

    # --- Reduced state: [pitch, pitch_rate, forward_velocity] ---
    # Freejoint indices: qpos[4] = qx ~ pitch (small-angle)
    # qvel[3] = angular velocity x (pitch rate)
    # qvel[0] = linear velocity x (forward)
    idx_theta = 4
    idx_theta_dot = 3
    idx_vx = 0

    S = np.zeros((3, nx))
    S[0, idx_theta] = 1.0
    S[1, idx_theta_dot] = 1.0
    S[2, idx_vx] = 1.0

    A_reduced = S @ A @ S.T

    # --- Inputs: wheel velocity actuators ---
    wheel_actuators = [2, 3]  # L_foot, R_foot
    B_reduced = S @ B
    B_reduced = B_reduced[:, wheel_actuators]

    print("A reduced")
    print(A_reduced)
    print("B reduced")
    print(B_reduced )

    #print_model_info(arena.mjcf_model, physics.model)

    A_c = (A_reduced - np.eye(3)) / dt
    B_c = B_reduced / dt
    print("Approx continuous-time A:\n", A_c)
    print("Approx continuous-time B:\n", B_c)

    #physics.data.ctrl[2] = 1.0  # left wheel
    #physics.data.ctrl[3] = 1.0  # right wheel
    #for _ in range(1):
    #    physics.step()
    #print(physics.data.qvel[0:3])  # forward velocity

    # --- LQR ---
    Q = np.diag([100, 10, 1])  # states: pitch, pitch_rate, forward velocity
    R = np.eye(2) * 0.1        # wheel velocity effort

    K, S_sol, E = control.dlqr(A_reduced, B_reduced, Q, R)
    print("LQR gain K:\n", K)

    
    DURATION  = 3   # seconds
    FRAMERATE = 60  # Hz

    model, data = physics.model, physics.data
    renderer = mujoco.Renderer(model.ptr, height=480, width=640)

    frames = []
    while data.time < DURATION:
        # Step the simulation.
        physics.step()

        # Render and save frames.
        if len(frames) < data.time * FRAMERATE:
            renderer.update_scene(data)
            pixels = renderer.render()
            frames.append(pixels)

    # Display video.
    media.show_video(frames, fps=FRAMERATE)

    



if __name__ == "__main__":
    main()
