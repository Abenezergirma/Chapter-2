import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting
from scipy import stats
from scipy import stats
from sklearn.metrics import mean_squared_error, mean_absolute_error
from scipy.interpolate import interp1d

# Use a nicer default aesthetic for plots
plt.rcParams['text.usetex'] = True
plt.rcParams['figure.figsize'] = (4, 4)  # Set default figure size to 4x4

class VisualizePrognostics:
    def __init__(self, pickle_path) -> None:
        self.prog_results = self.load_pickle_file(pickle_path)
        self.prog_times = self.prog_results.times
        self.voltage_python = self.get_prog_voltage()
        self.soc_python = self.get_SOC()
        self.num_traces = None  
        self.sample_time = None 
        self.current_input = None 
        self.voltage_matlab = None 
        self.SOC_matlab = None 
        self.soc_threshold = 0.30
        self.probability_sucess = None
        self.experiment_index = None 
        self.timeb_matlab = None
        self.overlay_plots = False 
        self.current_directory = os.path.dirname(os.path.abspath(__file__)) 
        self.plots_directory = os.path.join(self.current_directory,"ValidationResults/Pickles")
        self.llaTraj = None  # New attribute for 3D trajectory

    def load_pickle_file(self, pickle_path):
        with open(pickle_path, 'rb') as pickle_file:
            mc_results = pickle.load(pickle_file)
        return mc_results

    def get_prog_voltage(self):
        voltage = []
        for sim in range(len(self.prog_results.outputs)):
            voltage.append([self.prog_results.outputs[sim][time]['v'] for time in range(len(self.prog_results.outputs[sim]) - 2)])
        return voltage

    def get_SOC(self):
        soc = []
        for i in range(len(self.prog_results.event_states)):
            soc.append([self.prog_results.event_states[i][t]['EOD'] for t in range(len(self.prog_times) - 2)])
        return np.array(soc)

    def plot_3d_trajectory(self):
        """Plot a 3D trajectory of the aircraft from the llaTraj array."""
        if self.llaTraj is None:
            raise ValueError("LLA trajectory data not provided.")
        fig = plt.figure(figsize=(4, 4))
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(self.llaTraj[:, 0], self.llaTraj[:, 1], self.llaTraj[:, 2], color='royalblue', label='Aircraft Trajectory')
        ax.set_xlabel('Latitude')
        ax.set_ylabel('Longitude')
        ax.set_zlabel('Altitude (m)')
        ax.set_title('3D Aircraft Trajectory', fontsize=10)
        plt.legend(loc='best')
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/3D_Aircraft_Trajectory.png', dpi=300)
        plt.show()

    def plot_current_profile(self):
        """Plot the current profile of the battery."""
        if self.current_input is None:
            raise ValueError("Current input data not provided.")
        plt.figure(figsize=(3, 3))
        plt.plot(self.timeb_matlab, self.current_input, color='firebrick', linewidth=1.5, label='Current Profile')
        plt.xlabel('Time (s)', fontsize=10)
        plt.ylabel('Current (A)', fontsize=10)
        # plt.title('Battery Current Profile', fontsize=12)
        plt.legend(loc='best',fontsize=10)
        plt.grid(True, linestyle='--')
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/Current_Profile.pdf', dpi=300)
        plt.show()

    def plot_voltage_comparison(self):
        """Plot simulated and actual voltage trajectories."""
        time_stamp = int(len(self.current_input) * self.sample_time)
        plt.figure(figsize=(3, 3))
        for voltage in self.voltage_python:
            plt.plot(self.prog_times[:time_stamp], voltage[:time_stamp], linewidth=1.2, color='lightblue', alpha=0.8)
        plt.plot(self.timeb_matlab, self.voltage_matlab, linewidth=1, color='crimson', label='Actual Voltage')
        plt.xlabel('Time (s)', fontsize=10)
        plt.ylabel('Voltage (V)', fontsize=10)
        # plt.title('Simulated vs Actual Voltage', fontsize=12)
        plt.legend(loc='best',fontsize=10)
        plt.grid(True, linestyle='--')
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/Voltage_Comparison.pdf', dpi=300)
        plt.show()

    def plot_soc_comparison(self):
        """Plot simulated and actual SOC trajectories."""
        time_stamp = int(len(self.current_input) * self.sample_time)
        plt.figure(figsize=(3, 3))
        for soc in self.soc_python:
            plt.plot(self.prog_times[:time_stamp], soc[:time_stamp], linewidth=1, color='lightblue', alpha=0.8)
        plt.plot(self.timeb_matlab, self.SOC_matlab * 0.01, linewidth=1.8, color='crimson', label='Actual SOC')
        plt.xlabel('Time (s)', fontsize=10)
        plt.ylabel('SOC', fontsize=10)
        # plt.title('Simulated vs Actual SOC', fontsize=12)
        plt.legend(loc='best',fontsize=10)
        plt.grid(True, linestyle='--')
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/SOC_Comparison.pdf', dpi=300)
        plt.show()

    def save_side_by_side_plots(self):
        """Arrange three plots side by side for A4 paper format."""
        fig, axes = plt.subplots(1, 3, figsize=(10, 3.5))  # Three plots side by side for A4 paper
        time_stamp = int(len(self.current_input) * self.sample_time)

        # Plot current profile
        axes[0].plot(self.timeb_matlab, self.current_input, color='firebrick', linewidth=1.5, label='Current Profile')
        # axes[0].set_title('Current Profile', fontsize=10)
        axes[0].set_xlabel('Time (s)', fontsize=12)
        axes[0].set_ylabel('Current (A)', fontsize=12)
        axes[0].grid(True, linestyle='--')
        axes[0].legend(loc='best', fontsize=11)

        # Plot simulated and actual voltage
        for voltage in self.voltage_python:
            axes[1].plot(self.prog_times[:time_stamp], voltage[:time_stamp], linewidth=1, color='lightblue', alpha=0.5)
        axes[1].plot(self.prog_times[:time_stamp], voltage[:time_stamp], linewidth=1, color='lightblue', alpha=0.5, label='Simulated Voltage')
        axes[1].plot(self.timeb_matlab, self.voltage_matlab, linewidth=2, color='crimson', label='Actual Voltage')
        # axes[1].set_title('Simulated vs Actual Voltage', fontsize=10)
        axes[1].set_xlabel('Time (s)', fontsize=12)
        axes[1].set_ylabel('Voltage (V)', fontsize=12)
        axes[1].grid(True, linestyle='--')
        axes[1].legend(loc='best', fontsize=11)

        # Plot simulated and actual SOC
        for soc in self.soc_python:
            axes[2].plot(self.prog_times[:time_stamp], soc[:time_stamp], linewidth=1, color='lightblue', alpha=0.5)
        axes[2].plot(self.prog_times[:time_stamp], soc[:time_stamp], linewidth=1, color='lightblue', alpha=0.5, label='Simulated SoC')
        axes[2].plot(self.timeb_matlab, self.SOC_matlab * 0.01, linewidth=2, color='crimson', label='Actual SoC')
        # axes[2].set_title('Simulated vs Actual SOC', fontsize=10)
        axes[2].set_xlabel('Time (s)', fontsize=12)
        axes[2].set_ylabel('SoC', fontsize=12)
        axes[2].grid(True, linestyle='--')
        axes[2].legend(loc='best', fontsize=11)

        plt.tight_layout()
        plt.savefig(self.plots_directory + '/Current_Voltage_SOC_Comparison.pdf', dpi=300)
        plt.show()

        


    def compute_prognostics_accuracy(self):
        """Compute the accuracy of the prognostics compared to the actual flight data by interpolating actual data."""
        
        # Flatten timeb_matlab and ensure voltage_matlab and SOC_matlab are of the same length
        actual_voltage_trimmed = self.voltage_matlab.flatten()  # Flatten in case it's multidimensional
        actual_soc_trimmed = self.SOC_matlab.flatten()  # Flatten SOC as well
        
        # Flatten the time array (timeb_matlab)
        timeb_matlab_flat = self.timeb_matlab.flatten()  # Ensure time array is 1D
        
        # Truncate based on the shorter length between timeb_matlab_flat and voltage_matlab
        min_length_voltage = min(len(timeb_matlab_flat), len(actual_voltage_trimmed))
        timeb_matlab_trimmed_voltage = timeb_matlab_flat[:min_length_voltage]
        actual_voltage_trimmed = actual_voltage_trimmed[:min_length_voltage]

        min_length_soc = min(len(timeb_matlab_flat), len(actual_soc_trimmed))
        timeb_matlab_trimmed_soc = timeb_matlab_flat[:min_length_soc]
        actual_soc_trimmed = actual_soc_trimmed[:min_length_soc]

        # Debugging: Print lengths to ensure matching shapes
        print(f"Length of timeb_matlab_trimmed_voltage: {len(timeb_matlab_trimmed_voltage)}")
        print(f"Length of actual_voltage_trimmed: {len(actual_voltage_trimmed)}")
        print(f"Length of timeb_matlab_trimmed_soc: {len(timeb_matlab_trimmed_soc)}")
        print(f"Length of actual_soc_trimmed: {len(actual_soc_trimmed)}")

        # Compute average of the simulated trajectories
        avg_voltage = np.mean(self.voltage_python, axis=0)
        avg_soc = np.mean(self.soc_python, axis=0)

        # Get time array for the simulated data (prog_times)
        sim_time = self.prog_times[:len(avg_voltage)]

        # Interpolate actual data to match the simulated time steps
        voltage_interp_func = interp1d(timeb_matlab_trimmed_voltage, actual_voltage_trimmed, kind='linear', fill_value="extrapolate")
        actual_voltage_interp = voltage_interp_func(sim_time)

        soc_interp_func = interp1d(timeb_matlab_trimmed_soc, actual_soc_trimmed * 0.01, kind='linear', fill_value="extrapolate")
        actual_soc_interp = soc_interp_func(sim_time)

        # Now actual_voltage_interp and avg_voltage have the same shape
        # Calculate RMSE and MAE for voltage
        voltage_rmse = np.sqrt(mean_squared_error(actual_voltage_interp, avg_voltage))
        voltage_mae = mean_absolute_error(actual_voltage_interp, avg_voltage)

        # Calculate RMSE and MAE for SOC
        soc_rmse = np.sqrt(mean_squared_error(actual_soc_interp, avg_soc))
        soc_mae = mean_absolute_error(actual_soc_interp, avg_soc)

        # Print the accuracy metrics
        print(f"Voltage RMSE: {voltage_rmse:.4f}")
        print(f"Voltage MAE: {voltage_mae:.4f}")
        print(f"SOC RMSE: {soc_rmse:.4f}")
        print(f"SOC MAE: {soc_mae:.4f}")

        # Plot voltage error over time
        voltage_error = actual_voltage_interp - avg_voltage
        plt.figure(figsize=(4, 4))
        plt.plot(sim_time, voltage_error, color='royalblue', linewidth=1.5, label='Voltage Error')
        plt.xlabel('Time (s)', fontsize=10)
        plt.ylabel('Voltage Error (V)', fontsize=10)
        plt.title('Average Voltage Error Over Time', fontsize=12)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/Average_Voltage_Error.pdf', dpi=300)
        plt.show()

        # Plot SOC error over time
        soc_error = actual_soc_interp - avg_soc
        plt.figure(figsize=(4, 4))
        plt.plot(sim_time, soc_error, color='firebrick', linewidth=1.5, label='SOC Error')
        plt.xlabel('Time (s)', fontsize=10)
        plt.ylabel('SOC Error (%)', fontsize=10)
        plt.title('Average SOC Error Over Time', fontsize=12)
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(self.plots_directory + '/Average_SOC_Error.pdf', dpi=300)
        plt.show()
