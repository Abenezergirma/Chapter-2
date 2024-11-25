import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

class UAVTrajectoryVisualizer:
    def __init__(self, expt1_path, expt2_path):
        """
        Initialize the visualizer by loading the trajectories from the given paths.
        
        Parameters:
        - expt1_path: Path to the directory containing the first experiment trajectories.
        - expt2_path: Path to the directory containing the second experiment trajectories.
        """
        self.trajectories1 = self.load_trajectories(expt1_path)
        self.trajectories2 = self.load_trajectories(expt2_path)
        
        # Split and merge forward and return trajectories for both aircraft
        self.aircraft1_traj = self.merge_trajectories(self.trajectories1[:, 0, :], self.trajectories1[:, 1, :])
        self.aircraft2_traj = self.merge_trajectories(self.trajectories1[:, 2, :], self.trajectories1[:, 3, :])

    def load_trajectories(self, file_path):
        """
        Load trajectory data from the provided file path.
        """
        xTraj = sio.loadmat(file_path + '/xTraj.mat')['xTraj']
        yTraj = sio.loadmat(file_path + '/yTraj.mat')['yTraj']
        zTraj = sio.loadmat(file_path + '/zTraj.mat')['zTraj']
        return np.stack((xTraj.T, yTraj.T, zTraj.T), axis=2)

    def merge_trajectories(self, forward_traj, return_traj):
        """
        Merge the forward and return trajectories into one continuous trajectory.
        """
        return np.vstack((forward_traj, return_traj))

    def create_cube(self, xc, yc, zc, width, height):
        """
        Create a 3D cube representing the UAV's package.
        """
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

    def animate_uav(self, trajectory, output_file):
        """
        Animate the UAV trajectory along with the package below it and save as GIF.
        """
        fig = plt.figure(figsize=(8, 6))
        ax_traj = fig.add_subplot(111, projection='3d')
        
        # Set plot limits based on trajectory range
        ax_traj.set_xlim(np.min(trajectory[:, 0]), np.max(trajectory[:, 0]))
        ax_traj.set_ylim(np.min(trajectory[:, 1]), np.max(trajectory[:, 1]))
        ax_traj.set_zlim(np.min(trajectory[:, 2]), np.max(trajectory[:, 2]))

        ax_traj.set_title("Drone Package Delivery")
        ax_traj.set_xlabel("X")
        ax_traj.set_ylabel("Y")
        ax_traj.set_zlabel("Altitude (Z)")

        # Increase sizes to make elements more visible
        arm_length = 1.5  # Increase arm length for better visibility
        rotor_size = 8     # Increase rotor marker size

        # Initialize the UAV trajectory line and multirotor components
        line_traj, = ax_traj.plot([], [], [], 'b', lw=2)
        arm1, = ax_traj.plot([], [], [], color='black', lw=3)  # Increase arm line width
        arm2, = ax_traj.plot([], [], [], color='black', lw=3)

        rotor1, = ax_traj.plot([], [], [], color='red', marker='o', markersize=rotor_size)
        rotor2, = ax_traj.plot([], [], [], color='red', marker='o', markersize=rotor_size)
        rotor3, = ax_traj.plot([], [], [], color='red', marker='o', markersize=rotor_size)
        rotor4, = ax_traj.plot([], [], [], color='red', marker='o', markersize=rotor_size)

        # Increase package size for better visibility
        package_width = 10.0   # Increase width and depth of the package
        package_height = 3.0  # Increase height of the package

        # Create the package cube once and update its position
        cube_faces = self.create_cube(trajectory[0, 0], trajectory[0, 1], trajectory[0, 2] - arm_length - 1.2, package_width, package_height)
        cube = Poly3DCollection(cube_faces, facecolors='cyan', linewidths=2, edgecolors='blue', alpha=.85)
        ax_traj.add_collection3d(cube)

        def init():
            line_traj.set_data([], [])
            line_traj.set_3d_properties([])

            arm1.set_data([], [])
            arm1.set_3d_properties([])

            arm2.set_data([], [])
            arm2.set_3d_properties([])

            rotor1.set_data([], [])
            rotor1.set_3d_properties([])

            rotor2.set_data([], [])
            rotor2.set_3d_properties([])

            rotor3.set_data([], [])
            rotor3.set_3d_properties([])

            rotor4.set_data([], [])
            rotor4.set_3d_properties([])

            return line_traj, arm1, arm2, rotor1, rotor2, rotor3, rotor4

        def update(frame):
            xc, yc, zc = trajectory[frame]
            
            line_traj.set_data(trajectory[:frame, 0], trajectory[:frame, 1])
            line_traj.set_3d_properties(trajectory[:frame, 2])

            # Update the package position
            cube.set_verts(self.create_cube(xc, yc, zc - arm_length - 1.2, package_width, package_height))

            # Update UAV components
            arm1.set_data([xc - arm_length, xc + arm_length], [yc, yc])
            arm1.set_3d_properties([zc, zc])

            arm2.set_data([xc, xc], [yc - arm_length, yc + arm_length])
            arm2.set_3d_properties([zc, zc])

            rotor1.set_data([xc + arm_length], [yc])
            rotor1.set_3d_properties([zc])

            rotor2.set_data([xc - arm_length], [yc])
            rotor2.set_3d_properties([zc])

            rotor3.set_data([xc], [yc + arm_length])
            rotor3.set_3d_properties([zc])

            rotor4.set_data([xc], [yc - arm_length])
            rotor4.set_3d_properties([zc])

            return line_traj, arm1, arm2, rotor1, rotor2, rotor3, rotor4, cube

        ani = FuncAnimation(fig, update, frames=len(trajectory), init_func=init, blit=False, interval=50)
        
        # Save the animation as GIF
        # ani.save(output_file, writer='pillow', fps=30)
        
        plt.show()

    def run(self):
        """
        Run the animation for both aircraft independently.
        """
        self.animate_uav(self.aircraft1_traj, 'aircraft1_animation.gif')
        self.animate_uav(self.aircraft2_traj, 'aircraft2_animation.gif')


# Example usage
visualizer = UAVTrajectoryVisualizer('trajectories/expt_1', 'trajectories/expt_2')
visualizer.run()
