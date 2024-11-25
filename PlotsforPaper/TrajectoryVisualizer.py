import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os
from scipy.interpolate import interp1d
from scipy.ndimage import gaussian_filter1d
plt.rcParams['text.usetex'] = True
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
import os
import json
from scipy.interpolate import griddata

class TrajectoryVisualizer:
    def __init__(self, experiment1_dir, experiment2_dir, output_dir="plots", reference_point=None, R=6371000):
        """
        Initialize the visualizer with directories containing trajectories for two experiments.
        
        Parameters:
        experiment1_dir (str): Path to the directory containing x, y, z trajectories for experiment 1.
        experiment2_dir (str): Path to the directory containing x, y, z trajectories for experiment 2.
        output_dir (str): Directory where plots will be saved.
        smoothness (int): Number of points for interpolation (higher value = smoother).
        sigma (float): Standard deviation for Gaussian smoothing.
        num_waypoints (int): Number of waypoints to sample from the smoothed trajectory.
        """
        self.experiment1_dir = experiment1_dir
        self.experiment2_dir = experiment2_dir
        self.output_dir = output_dir
        
        # Create output directory if it doesn't exist
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
        
        # Load trajectories for both experiments
        self.trajectories1 = self.load_trajectories(self.experiment1_dir)
        self.trajectories2 = self.load_trajectories(self.experiment2_dir)
        
        self.reference_point = reference_point  # (longitude, latitude)
        self.R = R
        # Conversion factors for latitude and longitude
        self.meters_per_degree_lat = 111000  # Approximate meters per degree latitude
        if reference_point:
            self.meters_per_degree_lon = self.meters_per_degree_lat * np.cos(np.deg2rad(self.reference_point[1]))  # Adjust for longitude

    def load_trajectories(self, directory):
        """
        Load x, y, z trajectory data from .mat files.
        
        Parameters:
        directory (str): Path to the directory containing xTraj.mat, yTraj.mat, and zTraj.mat files.
        
        Returns:
        np.ndarray: Trajectories array with shape (N, 4, 3), where N is the number of points,
                    4 is the number of aircraft, and 3 corresponds to (x, y, z) positions.
        """
        xTraj = sio.loadmat(os.path.join(directory, 'xTraj.mat'))['xTraj']
        yTraj = sio.loadmat(os.path.join(directory, 'yTraj.mat'))['yTraj']
        zTraj = sio.loadmat(os.path.join(directory, 'zTraj.mat'))['zTraj']
        
        # Stack them along the third dimension as (N, 4, 3) format
        trajectories = np.stack((xTraj.T, yTraj.T, zTraj.T), axis=2)
        
        # Add takeoff and landing segments for each trajectory
        trajectories = self.add_takeoff_landing_segments(trajectories)
        
        return trajectories

    def add_takeoff_landing_segments(self, trajectories):
        """
        Add takeoff and landing segments to the trajectories.
        
        Parameters:
        trajectories (np.ndarray): Original trajectory array with shape (N, 4, 3).
        
        Returns:
        np.ndarray: Modified trajectory array with takeoff and landing segments.
        """
        N, M, _ = trajectories.shape
        modified_trajectories = []

        for i in range(M):
            # Extract the current trajectory for the aircraft
            trajectory = trajectories[:, i, :]
            
            # Takeoff segment: [x0, y0, 60] -> [x0, y0, 100]
            takeoff_segment = np.array([[trajectory[0, 0], trajectory[0, 1], 60], 
                                        [trajectory[0, 0], trajectory[0, 1], 100]])
            
            # Landing segment: [xN, yN, 100] -> [xN, yN, 60]
            landing_segment = np.array([[trajectory[-1, 0], trajectory[-1, 1], 100], 
                                        [trajectory[-1, 0], trajectory[-1, 1], 60]])
            
            # Concatenate the takeoff, original trajectory, and landing segments
            full_trajectory = np.vstack((takeoff_segment, trajectory, landing_segment))
            modified_trajectories.append(full_trajectory)
        
        return np.stack(modified_trajectories, axis=1)

    def plot_trajectories(self, original1, original2, aircraft_idx):
        """
        Plot the original trajectories for two experiments.
        
        Parameters:
        original1 (np.ndarray): Original trajectory for experiment 1.
        original2 (np.ndarray): Original trajectory for experiment 2.
        aircraft_idx (int): Index of the aircraft for labeling purposes.
        """
        fig = plt.figure(figsize=(8.27, 11.69))  # A4 size in inches
        ax = fig.add_subplot(111, projection='3d')

        # Plot trajectories for experiment 1
        ax.plot(original1[:, 0], original1[:, 1], original1[:, 2], 'r--', label='Experiment 1', linewidth=2)
        # Plot trajectories for experiment 2
        ax.plot(original2[:, 0], original2[:, 1], original2[:, 2], 'b-', label='Experiment 2', linewidth=2)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title(f'Aircraft {aircraft_idx} Trajectory Comparison')
        ax.legend()
        ax.grid(True)
        
        # Save the plot in PNG format
        plt.savefig(f"{self.output_dir}/aircraft_{aircraft_idx}_comparison.png", bbox_inches='tight')
        # Save the plot in PDF format
        plt.savefig(f"{self.output_dir}/aircraft_{aircraft_idx}_comparison.pdf", bbox_inches='tight')
        plt.close()
        
    def plot_trajectories_independently(self):
        """
        Plot the original trajectories for all aircraft independently, saving each as a separate figure.
        The plots are designed for a 2x2 arrangement on A4 paper.
        """
        num_aircraft = self.trajectories1.shape[1]  # Number of aircraft
        
        # Loop through each aircraft and save each plot independently
        for i in range(num_aircraft):
            original1_trajectory = self.trajectories1[:, i, :]
            original2_trajectory = self.trajectories2[:, i, :]

            # Create figure with A4 size
            fig = plt.figure(figsize=(8.27/2.5,8.27/2.5))  # A4 paper size in inches (portrait)
            ax = fig.add_subplot(111, projection='3d')

            # Plot trajectories for experiment 1
            ax.plot(original1_trajectory[:, 0], original1_trajectory[:, 1], original1_trajectory[:, 2], 
                    'r--', label='Baseline', linewidth=2)
            # Plot trajectories for experiment 2
            ax.plot(original2_trajectory[:, 0], original2_trajectory[:, 1], original2_trajectory[:, 2], 
                    'b-', label='Energy Efficient', linewidth=2)

            ax.set_xlabel('X', fontsize=8)
            ax.set_ylabel('Y', fontsize=8)
            ax.set_zlabel('Z', fontsize=8)
            # ax.set_title(f'Aircraft {i + 1} Trajectory Comparison')
            ax.legend(prop={'size': 8})
            ax.grid(True)
            
            # Save the plot independently
            # plt.savefig(f"{self.output_dir}/aircraft_{i + 1}_trajectory_comparison.png", bbox_inches='tight')
            plt.savefig(f"{self.output_dir}/aircraft_{i + 1}_trajectory_comparison.pdf", bbox_inches='tight')
            plt.close()



    def visualize_all(self):
        """
        Generate and save plots for all aircraft for both experiments.
        """
        N, M, _ = self.trajectories1.shape
        
        # Create a 2x2 grid for the plots
        fig, axes = plt.subplots(2, 2, subplot_kw={'projection': '3d'}, figsize=(8.27, 11.69))  # A4 paper size

        for i in range(M):
            original1_trajectory = self.trajectories1[:, i, :]
            original2_trajectory = self.trajectories2[:, i, :]

            ax = axes[i // 2, i % 2]
            
            # Plot trajectories for experiment 1
            ax.plot(original1_trajectory[:, 0], original1_trajectory[:, 1], original1_trajectory[:, 2], 'r--', label='Experiment 1', linewidth=2)
            # Plot trajectories for experiment 2
            ax.plot(original2_trajectory[:, 0], original2_trajectory[:, 1], original2_trajectory[:, 2], 'b-', label='Experiment 2', linewidth=2)

            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
            ax.set_title(f'Aircraft {i+1}')
            ax.legend()

        # Adjust layout to make sure everything fits nicely on the A4 page
        plt.tight_layout()
        plt.savefig(f"{self.output_dir}/all_aircraft_comparison.png", bbox_inches='tight')
        plt.savefig(f"{self.output_dir}/all_aircraft_comparison.pdf", bbox_inches='tight')
        plt.close()

    def xy_to_latlon(self, x, y):
        """
        Convert x, y coordinates back to latitude and longitude.
        
        Parameters:
        x (float): x coordinate in meters.
        y (float): y coordinate in meters.
        
        Returns:
        tuple: (longitude, latitude) corresponding to the x, y values.
        """
        lon = x / self.meters_per_degree_lon + self.reference_point[0]
        lat = y / self.meters_per_degree_lat + self.reference_point[1]
        return lon, lat

    def convert_to_geojson(self):
        """
        Convert smoothed x, y trajectories of all aircraft to a single GeoJSON file for QGIS.
        """
        geojson = {
            "type": "FeatureCollection",
            "features": []
        }

        # Loop through each aircraft's trajectory
        num_aircraft = self.trajectories1.shape[1]  # Number of aircraft
        for aircraft_idx in range(num_aircraft):
            # Extract the smoothed trajectory for the current aircraft (x, y only)
            smoothed_trajectory = self.trajectories1[:, aircraft_idx, :2]  # Get only the x, y dimensions

            # Convert each x, y coordinate to lat, lon
            coordinates = []
            for point in smoothed_trajectory:
                x, y = point[:2]  # Get the x and y coordinates
                lon, lat = self.xy_to_latlon(x, y)
                coordinates.append([lon, lat])

            # Create the GeoJSON feature for this aircraft
            feature = {
                "type": "Feature",
                "geometry": {
                    "type": "LineString",
                    "coordinates": coordinates
                },
                "properties": {
                    "aircraft": f"Aircraft {aircraft_idx + 1}"
                }
            }

            # Append the feature for this aircraft
            geojson["features"].append(feature)

        # Save the GeoJSON file containing trajectories for all aircraft
        geojson_path = os.path.join(self.output_dir, "all_aircraft_trajectories.geojson")
        with open(geojson_path, 'w') as f:
            json.dump(geojson, f, indent=4)
            
    def combine_trajectories(self):
        """
        Combine the correct back-and-forth trajectories for two aircraft (1 with 2, and 3 with 4).
        Add takeoff and landing segments for the combined trajectories.
        
        Returns:
        combined_trajectories: A list of combined trajectories for aircraft 1 and 2.
        """
        combined_trajectories = []

        # Combine trajectory 1 and trajectory 2 (aircraft 1, back-and-forth flight)
        trajectory_1 = self.trajectories1[:, 0, :]  # Trajectory 1
        trajectory_2 = self.trajectories1[:, 1, :]  # Trajectory 2
        combined_trajectory_1 = np.vstack((trajectory_1, trajectory_2))  # Combine the two segments
        combined_trajectory_1_with_segments = self.add_takeoff_landing_segments(combined_trajectory_1)
        combined_trajectories.append(combined_trajectory_1_with_segments)

        # Combine trajectory 3 and trajectory 4 (aircraft 2, back-and-forth flight)
        trajectory_3 = self.trajectories1[:, 2, :]  # Trajectory 3
        trajectory_4 = self.trajectories1[:, 3, :]  # Trajectory 4
        combined_trajectory_2 = np.vstack((trajectory_3, trajectory_4))  # Combine the two segments
        combined_trajectory_2_with_segments = self.add_takeoff_landing_segments(combined_trajectory_2)
        combined_trajectories.append(combined_trajectory_2_with_segments)

        return combined_trajectories

    def add_takeoff_landing_segments(self, trajectory):
        """
        Add takeoff and landing segments to a given trajectory.
        
        Parameters:
        trajectory (np.ndarray): Trajectory array of shape (N, 3) representing (x, y, z) coordinates 
                                or (N, M, 3) for multiple trajectories.
        
        Returns:
        np.ndarray: Modified trajectory array with takeoff and landing segments added.
        """
        if trajectory.ndim == 3:
            # For multiple trajectories (3D array)
            N, M, _ = trajectory.shape
            modified_trajectories = []
            
            for i in range(M):
                # Extract the current trajectory for the aircraft
                single_trajectory = trajectory[:, i, :]
                
                # Add takeoff and landing segments
                takeoff_segment = np.array([[single_trajectory[0, 0], single_trajectory[0, 1], 60], 
                                            [single_trajectory[0, 0], single_trajectory[0, 1], 100]])
                landing_segment = np.array([[single_trajectory[-1, 0], single_trajectory[-1, 1], 100], 
                                            [single_trajectory[-1, 0], single_trajectory[-1, 1], 60]])
                
                # Concatenate the segments
                full_trajectory = np.vstack((takeoff_segment, single_trajectory, landing_segment))
                modified_trajectories.append(full_trajectory)
            
            # Stack modified trajectories along the second dimension to maintain shape (N + 4, M, 3)
            return np.stack(modified_trajectories, axis=1)
        
        elif trajectory.ndim == 2:
            # For a single trajectory (2D array)
            N, _ = trajectory.shape
            
            # Add takeoff and landing segments
            takeoff_segment = np.array([[trajectory[0, 0], trajectory[0, 1], 60], 
                                        [trajectory[0, 0], trajectory[0, 1], 100]])
            landing_segment = np.array([[trajectory[-1, 0], trajectory[-1, 1], 100], 
                                        [trajectory[-1, 0], trajectory[-1, 1], 60]])
            
            # Concatenate the segments
            full_trajectory = np.vstack((takeoff_segment, trajectory, landing_segment))
            return full_trajectory

        else:
            raise ValueError("Trajectory must be a 2D or 3D array.")

            
    def plot_combined_trajectories(self):
        """
        Plot and save the combined trajectories for aircraft 1 and 2 after adding takeoff and landing segments.
        Both experiments will be plotted for comparison.
        """
        # Combine trajectories from experiment 1
        combined_trajectories_expt1 = []
        trajectory_1_expt1 = np.vstack((self.trajectories1[:, 0, :], self.trajectories1[:, 1, :]))  # Combine trajectory 1 and 2
        combined_trajectories_expt1.append(self.add_takeoff_landing_segments(trajectory_1_expt1))
        
        trajectory_2_expt1 = np.vstack((self.trajectories1[:, 2, :], self.trajectories1[:, 3, :]))  # Combine trajectory 3 and 4
        combined_trajectories_expt1.append(self.add_takeoff_landing_segments(trajectory_2_expt1))
        
        # Combine trajectories from experiment 2
        combined_trajectories_expt2 = []
        trajectory_1_expt2 = np.vstack((self.trajectories2[:, 0, :], self.trajectories2[:, 1, :]))  # Combine trajectory 1 and 2
        combined_trajectories_expt2.append(self.add_takeoff_landing_segments(trajectory_1_expt2))
        
        trajectory_2_expt2 = np.vstack((self.trajectories2[:, 2, :], self.trajectories2[:, 3, :]))  # Combine trajectory 3 and 4
        combined_trajectories_expt2.append(self.add_takeoff_landing_segments(trajectory_2_expt2))
        
        # Plot each combined trajectory for both experiments independently
        for i, (trajectory_expt1, trajectory_expt2) in enumerate(zip(combined_trajectories_expt1, combined_trajectories_expt2)):
            fig = plt.figure(figsize=(4, 4))  # Slightly larger figure for better visualization
            fig.subplots_adjust(left=-0.2, right=0.95, top=0.9, bottom=0.15)  # Squeeze plotting area

            ax = fig.add_subplot(111, projection='3d')

            # Plot combined trajectory from experiment 1
            ax.plot(trajectory_expt1[:, 0], trajectory_expt1[:, 1], trajectory_expt1[:, 2] - 60,
                    'r--', label='Baseline', linewidth=2)

            # Plot combined trajectory from experiment 2
            ax.plot(trajectory_expt2[:, 0], trajectory_expt2[:, 1], trajectory_expt2[:, 2] - 60,
                    'b-', label='Energy-Efficient', linewidth=2)

            # Markers for the depot and delivery points
            depot_marker = 'o'
            delivery_marker = 's'
            depot_color = 'green'
            delivery_color = 'purple'

            # Mark depot (same for both trajectories)
            ax.scatter(trajectory_expt1[0, 0], trajectory_expt1[0, 1], trajectory_expt1[0, 2] - 60,
                    color=depot_color, s=50, marker=depot_marker, label='Depot')
            ax.scatter(trajectory_expt1[-1, 0], trajectory_expt1[-1, 1], trajectory_expt1[-1, 2] - 60,
                    color=depot_color, s=50, marker=depot_marker)
            # ax.text(trajectory_expt1[0, 0], trajectory_expt1[0, 1], trajectory_expt1[0, 2] - 60,
            #         'Depot', color=depot_color, fontsize=8)

            # Mark delivery points based on pre-merged trajectories
            if i == 0:  # First mission
                delivery_point = self.trajectories1[-1, 0, :]  # End of trajectory 1 before merging
            else:  # Second mission
                delivery_point = self.trajectories1[-1, 2, :]  # End of trajectory 3 before merging
            
            ax.scatter(delivery_point[0], delivery_point[1], delivery_point[2] - 60,
                    color=delivery_color, s=50, marker=delivery_marker, label='Delivery Point')

            # Enhance the plot for visual attractiveness and adjust axis ticks and label padding
            ax.set_xlabel('X (meters)', fontsize=10, labelpad=-2)
            ax.set_ylabel('Y (meters)', fontsize=10, labelpad=-2)
            
            # Manually adjust the z-axis label padding and bring tick labels closer
            ax.set_zlabel('Z (meters)', fontsize=10, labelpad=-9)
            ax.tick_params(axis='x', pad=-2)  # Bring x-axis tick labels closer
            ax.tick_params(axis='y', pad=-2)  # Bring y-axis tick labels closer
            ax.tick_params(axis='z', pad=-2)  # Bring z-axis tick labels closer

            ax.legend(prop={'size': 10}, loc='upper right')
            ax.grid(True)

            # Adjust the view angle for better visualization
            ax.view_init(elev=30, azim=120)
            ax.xaxis._axinfo['grid'].update(color='grey', linestyle='--', linewidth=0.5)
            ax.yaxis._axinfo['grid'].update(color='grey', linestyle='--', linewidth=0.5)
            ax.zaxis._axinfo['grid'].update(color='grey', linestyle='--', linewidth=0.5)

            # Save the plot independently for each combined trajectory
            plt.tight_layout()
            plt.savefig(f"{self.output_dir}/combined_trajectory_comparison_{i + 1}.pdf", bbox_inches='tight')
            plt.close()





    def meters_to_latlon(self, x, y, center_lat, center_lon):
        # Constants
        meters_per_degree_lat = 111319
        meters_per_degree_lon = meters_per_degree_lat * np.cos(np.radians(center_lat))

        # Convert
        delta_lat = y / meters_per_degree_lat
        delta_lon = x / meters_per_degree_lon

        return center_lat + delta_lat, center_lon + delta_lon

    def lat_lon_to_meters(self, lat, lon, center_lat, center_lon):
        # Earth's radius in meters
        R = 6371000
        # Convert latitude and longitude differences to radians
        delta_lat = lat - center_lat
        delta_lon = lon - center_lon
        # Approximate conversion to meters
        m_lat = delta_lat * R
        m_lon = delta_lon * R * np.cos(center_lat)
        return m_lat, m_lon
    
   


# Example usage:
visualizer = TrajectoryVisualizer("trajectories/expt_1", "trajectories/expt_2",  reference_point=(-96.8291, 33.1492))
# smoothed_trajectory = visualizer.load_trajectories(directory)
visualizer.visualize_all()
visualizer.convert_to_geojson()
visualizer.plot_trajectories_independently()

# Define the Frisco city coordinates
lower_left = [-96.92379, 33.079172]
upper_right = [-96.730499, 33.223754]

visualizer.plot_combined_trajectories()
