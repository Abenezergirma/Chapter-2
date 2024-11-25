import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import scipy.io as sio

# Helper function to create a cube for the package
def create_cube(xc, yc, zc, width, height):
    r_w = [-width / 2, width / 2]
    r_h = [-height / 2, height / 2]
    vertices = np.array([[xc + x, yc + y, zc + z] for x in r_w for y in r_w for z in r_h])
    faces = [
        [vertices[0], vertices[1], vertices[3], vertices[2]],
        [vertices[4], vertices[5], vertices[7], vertices[6]],
        [vertices[0], vertices[1], vertices[5], vertices[4]],
        [vertices[2], vertices[3], vertices[7], vertices[6]],
        [vertices[0], vertices[2], vertices[6], vertices[4]],
        [vertices[1], vertices[3], vertices[7], vertices[5]],
    ]
    return faces

# Load trajectory data (modify paths as needed)
def load_trajectories(file_path):
    xTraj = sio.loadmat(file_path + '/xTraj.mat')['xTraj']
    yTraj = sio.loadmat(file_path + '/yTraj.mat')['yTraj']
    zTraj = sio.loadmat(file_path + '/zTraj.mat')['zTraj']
    return np.stack((xTraj.T, yTraj.T, zTraj.T), axis=2)

# Load trajectories for both aircraft
trajectories1 = load_trajectories('trajectories/expt_1')
trajectories2 = load_trajectories('trajectories/expt_2')

# Split the trajectories for two different aircraft
aircraft1_traj1 = trajectories1[:, 0, :]  # Forward trajectory for aircraft 1
aircraft1_traj2 = trajectories1[:, 1, :]  # Return trajectory for aircraft 1
aircraft2_traj1 = trajectories1[:, 2, :]  # Forward trajectory for aircraft 2
aircraft2_traj2 = trajectories1[:, 3, :]  # Return trajectory for aircraft 2

# Create the figure and axes for each aircraft animation
def create_animation(aircraft_traj1, aircraft_traj2, output_file):
    fig = plt.figure(figsize=(10, 8))
    ax_traj = fig.add_subplot(111, projection='3d')

    # Set plot limits to simulate an outdoor environment
    ax_traj.set_xlim(-2, 2)
    ax_traj.set_ylim(-2, 2)
    ax_traj.set_zlim(0, 20)

    # Add ground plane and sky-like background
    ground_x, ground_y = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(-2, 2, 100))
    ground_z = np.zeros_like(ground_x)
    ax_traj.plot_surface(ground_x, ground_y, ground_z, color='green', alpha=0.3)
    ax_traj.w_xaxis.pane.set_facecolor((0.5, 0.7, 1.0, 1.0))
    ax_traj.w_yaxis.pane.set_facecolor((0.5, 0.7, 1.0, 1.0))
    
    # Initialize lines and packages for two UAVs
    line_traj1, = ax_traj.plot([], [], [], 'b', lw=2)
    line_traj2, = ax_traj.plot([], [], [], 'r', lw=2)

    arm_length = 0.1
    package_width = 0.2
    package_height = 0.3

    # Multirotor arms and rotors for both UAVs
    arm1_traj1, = ax_traj.plot([], [], [], color='black', lw=2)
    arm2_traj1, = ax_traj.plot([], [], [], color='black', lw=2)
    rotor1_traj1, = ax_traj.plot([], [], [], color='red', marker='o', markersize=5)
    rotor2_traj1, = ax_traj.plot([], [], [], color='red', marker='o', markersize=5)

    arm1_traj2, = ax_traj.plot([], [], [], color='black', lw=2)
    arm2_traj2, = ax_traj.plot([], [], [], color='black', lw=2)
    rotor1_traj2, = ax_traj.plot([], [], [], color='red', marker='o', markersize=5)
    rotor2_traj2, = ax_traj.plot([], [], [], color='red', marker='o', markersize=5)

    def init():
        line_traj1.set_data([], [])
        line_traj1.set_3d_properties([])
        line_traj2.set_data([], [])
        line_traj2.set_3d_properties([])
        return line_traj1, line_traj2

    def update(frame):
        # Update UAV 1 trajectory
        line_traj1.set_data(aircraft_traj1[:frame, 0], aircraft_traj1[:frame, 1])
        line_traj1.set_3d_properties(aircraft_traj1[:frame, 2])
        
        # Update UAV 2 trajectory
        line_traj2.set_data(aircraft_traj2[:frame, 0], aircraft_traj2[:frame, 1])
        line_traj2.set_3d_properties(aircraft_traj2[:frame, 2])

        # Update positions of the packages (cubes)
        xc1, yc1, zc1 = aircraft_traj1[frame]
        cube_faces1 = create_cube(xc1, yc1, zc1 - arm_length - package_height / 2, package_width, package_height)
        ax_traj.collections = [coll for coll in ax_traj.collections if not isinstance(coll, Poly3DCollection)]
        ax_traj.add_collection3d(Poly3DCollection(cube_faces1, facecolors='cyan', linewidths=1, edgecolors='blue', alpha=0.75))

        xc2, yc2, zc2 = aircraft_traj2[frame]
        cube_faces2 = create_cube(xc2, yc2, zc2 - arm_length - package_height / 2, package_width, package_height)
        ax_traj.add_collection3d(Poly3DCollection(cube_faces2, facecolors='orange', linewidths=1, edgecolors='blue', alpha=0.75))

        return line_traj1, line_traj2

    ani = FuncAnimation(fig, update, frames=len(aircraft_traj1), init_func=init, blit=False, interval=100)

    # Save the animation as a file
    ani.save(output_file, writer='ffmpeg')

# Generate animations for the two aircraft
create_animation(aircraft1_traj1, aircraft1_traj2, 'aircraft1_animation.mp4')
create_animation(aircraft2_traj1, aircraft2_traj2, 'aircraft2_animation.mp4')
