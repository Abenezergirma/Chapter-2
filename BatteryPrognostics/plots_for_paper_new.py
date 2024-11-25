import os
import pickle 
import glob
import warnings
import numpy as np
import matplotlib.pyplot as plt
import collections
import pandas as pd
from scipy import stats
from scipy.stats import norm
from progpy.metrics import prob_success
# from sklearn.preprocessing import normalize 
from pathlib import Path
import sys 
from scipy.io import loadmat

plt.rcParams['text.usetex'] = True

class PlotsPaper:
    def __init__(self, pickle_path) -> None:
        self.prog_results = self.load_pickle_files(pickle_path)
        # print(self.prog_results['mc_results_1'].event_states[0])
        self.prog_times = None #self.get_prog_times()
        self.voltage_python = None #self.get_prog_voltage_forall_pickles()
        self.soc_python = None #self.get_SOC_forall_pickles()
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
        self.current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
        self.plots_directory = os.path.join(self.current_directory, "..", "BatteryPrognosticsResults/Pickles")
        
    def get_prog_times(self):
        prog_time = []
        loaded_results = self.prog_results
        for result_key in loaded_results:  
            prog_time.append(loaded_results[result_key].times)
        return prog_time
        
    def load_pickle_files(self, folder_path):
        mc_results = {}  # a dictionary to store the loaded pickle files with different names
        pickle_files = glob.glob(os.path.join(folder_path, '*.pkl'))
        for i, pickle_path in enumerate(pickle_files):
            with open(pickle_path, 'rb') as pickle_file:
                mc_results[f'mc_results_{i+1}'] = pickle.load(pickle_file)
        return mc_results
    
    def get_prog_voltage_forall_pickles(self):
        voltage_profiles = []
        
        loaded_results = self.prog_results
        
        for result_key in loaded_results:           
            prog_results = loaded_results[result_key]
            voltage = []
            for sim in range(len(prog_results.outputs)):
                voltage.append([])
                for time in range(len(prog_results.outputs[sim]) - 4):
                    voltage[sim].append(prog_results.outputs[sim][time]['v'])

            voltage_profiles.append({result_key: voltage})

        return voltage_profiles
    
    def get_SOC_forall_pickles(self):
        loaded_results = self.prog_results
        SOC_profiles = []

        for result_key in loaded_results:
            
            prog_results = loaded_results[result_key]
            soc = []
            for i in range(len(prog_results.event_states)):
                soc.append([])
                for t in range(len(prog_results.times)-4):
                    # print(len(prog_results.times)-1)
                    # print(prog_results.event_states[i][t]['EOD'])                  
                    soc[i].append(prog_results.event_states[i][t]['EOD'])

            SOC_profiles.append({result_key: np.array(soc)})
        # print(prog_results)
        return SOC_profiles
    
    def plot_soc_predictions(self):
        self.soc_python = self.get_SOC_forall_pickles()
        num_experiments = len(self.soc_python)
        num_rows = 2  # Number of rows in the subplot grid
        num_cols = 3  # Number of columns in the subplot grid
        # warnings.filterwarnings("ignore", category=MatplotlibDeprecationWarning)

        
        plt.subplots_adjust(bottom=0.15)
        plt.rcParams["figure.figsize"] = (6, 8)  # Adjust figure size for multiple subplots
        # plt.grid()

        for i, profile_dict in enumerate(self.soc_python):
            ax = plt.subplot(num_rows, num_cols, i + 1)  # Create a new subplot for each experiment
            
            mean_SOC = []  # Initialize list for mean SOC
            
            for profile_name, soc_profile in profile_dict.items():
                # Extract the last element of each profile as SOC
                SOC = [profile[-1] for profile in soc_profile]

                # Compute statistics for SOC
                mu = np.mean(SOC)
                sigma = np.std(SOC)

                # Generate the bins for the histogram
                # (values, bins, _) = ax.hist(SOC, bins=100, density=True, label=f"Histogram {i+1}", alpha=0.6, color='lightblue', edgecolor='black', linewidth=1.5, range=(0, 1))
                (values, bins, _) = ax.hist(SOC, bins=100, density=True, label=f"Histogram {i+1}", alpha=0.6, color='lightblue', edgecolor='lightblue', linewidth=1.5, range=(0, 1))

                # Generate the PDF
                bin_centers = 0.5 * (bins[1:] + bins[:-1])
                pdf = stats.norm.pdf(x=bin_centers, loc=mu, scale=sigma)

                # Plot the PDF within the current subplot
                ax.plot(bin_centers, pdf, label=f"Gaussian Curve {i+1}", color='green', linestyle='-', linewidth=1.3, alpha=0.7)

                # Append the mean SOC for this profile to the list
                mean_SOC.append(mu)

            ax.axvline(x=self.soc_threshold, color='red', label="SOC threshold", linewidth=2, linestyle='--')
            ax.legend(prop={'size': 10}, loc='upper right')
            ax.set_xlabel(f'SOC at {len(soc_profile[0])} sec', fontsize=10)
            ax.set_ylabel('Density', fontsize=10)
            # ax.set_title(f'Battery SOC Prediction - Aircraft {i+1}', fontsize=12)
            ax.grid(True, linestyle='--', alpha=0.7)

            # Compute and display the mean SOC for this experiment
            mean_experiment_SOC = np.mean(mean_SOC)
            ax.text(0.05, 0.9, f"Mean SOC: {mean_experiment_SOC:.3f}", transform=plt.gca().transAxes, fontsize=10, color='blue')

        plt.tight_layout()  # Adjust subplot layout
        plt.savefig(os.path.join(self.current_directory, 'Battery_SOC_predictions.pdf'), format='pdf', dpi=140)
        plt.show()
        plt.close()
        
    def plot_soc_predictions_save_independetly(self):
        self.soc_python = self.get_SOC_forall_pickles()
        wind_label = ['No','Medium', 'Low', 'Strong']    


        for i, profile_dict in enumerate(self.soc_python):
            if i == 3:

                
                mean_SOC = []
                timeStamps = [15,180,270,480]
                for k in timeStamps:
                    fig, ax = plt.subplots(figsize=(4, 4))  # Adjust the size of each plot
                    # plt.grid()

                    for profile_name, soc_profile in profile_dict.items():
    
                        SOC = [profile[k] for profile in soc_profile]
                        mu = np.mean(SOC)
                        sigma = np.std(SOC)

                        (values, bins, _) = ax.hist(SOC, bins=100, density=True, label=f"Histogram", alpha=0.6, color='lightblue', linewidth=1.5, range=(0, 1))

                        bin_centers = 0.5 * (bins[1:] + bins[:-1])
                        pdf = stats.norm.pdf(x=bin_centers, loc=mu, scale=sigma)
                        ax.plot(bin_centers, pdf, label=f"Gaussian Curve", color='green', linestyle='-', linewidth=1.3, alpha=0.7)

                        mean_SOC.append(mu)

                    ax.axvline(x=self.soc_threshold, color='red', label="SOC threshold", linewidth=2, linestyle='--')
                    ax.legend(prop={'size': 8}, loc='upper right')  # Smaller legend font size
                    ax.set_xlabel(f'SOC at {k} sec', fontsize=10)  # Smaller font size
                    ax.set_ylabel('Density', fontsize=10)  # Smaller font size
                    ax.set_title(f'Battery SOC Prediction - {wind_label[i]} Wind', fontsize=10)  # Slightly larger title font size
                    ax.grid(True, linestyle='--', alpha=0.7)

                    mean_experiment_SOC = np.mean(mean_SOC)
                    ax.text(0.05, 0.9, f"Mean SOC: {mean_experiment_SOC:.3f}", transform=ax.transAxes, fontsize=10, color='blue')  # Smaller font size

                    # plt.tight_layout()
                    # ax.show()
                    filename = f'Battery_SOC_Prediction_Aircraft_{k+2}.png'
                    plt.savefig(os.path.join(self.current_directory, filename), format='png')
                    plt.close(fig)
        
    def plot_voltage(self):
        self.voltage_python = self.get_prog_voltage_forall_pickles()
        self.prog_times = self.get_prog_times()
        num_experiments = len(self.voltage_python)
        num_rows = 2  # Number of rows in the subplot grid
        num_cols = 3  # Number of columns in the subplot grid

        plt.subplots_adjust(bottom=0.2)
        plt.rcParams["figure.figsize"] = (12, 8)  # Adjust figure size for multiple subplots
        # plt.grid()

        for i, voltage_dict in enumerate(self.voltage_python):
            plt.subplot(num_rows, num_cols, i + 1)  # Create a new subplot for each experiment
            # print(self.prog_times[i])

            for voltage_experiment in voltage_dict.values():
                time_stamp = len(self.prog_times[i])-4
                for voltage in voltage_experiment:
                    plt.plot(self.prog_times[i][0:time_stamp], voltage[0:time_stamp], linewidth=1, color='blue')

            plt.xlabel('time (sec)', fontsize=10)
            plt.ylabel('voltage', fontsize=10)
            plt.title(f'Voltage Trajectories - Aircraft {i + 1}', fontsize=12)
            plt.grid(True, linestyle='--', alpha=0.7)

        plt.tight_layout()  # Adjust subplot layout
        plt.savefig(os.path.join(self.current_directory, 'Battery_Voltage_trajectories.png'), format='png', dpi=140)
        plt.show()
        plt.close()

    def plot_voltage_independently(self):
        self.voltage_python = self.get_prog_voltage_forall_pickles()
        self.prog_times = self.get_prog_times()
        
        # plt.subplots_adjust(bottom=0.2)
        plt.rcParams["figure.figsize"] = (4, 4)  # Adjust figure size for multiple subplots
        # plt.grid()
        wind_label = ['No','Medium', 'Low', 'Strong']    

        for i, voltage_dict in enumerate(self.voltage_python):
            # plt.subplot(num_rows, num_cols, i + 1)  # Create a new subplot for each experiment
            # print(self.prog_times[i])

            for voltage_experiment in voltage_dict.values():
                time_stamp = len(self.prog_times[i])-5
                for voltage in voltage_experiment:
                    print(len(self.prog_times[i][0:time_stamp]))
                    print(len(voltage[0:time_stamp]))
                    plt.plot(self.prog_times[i][0:time_stamp], voltage[0:time_stamp], linewidth=1, color='blue')

            plt.xlabel('Time (sec)', fontsize=10)
            plt.ylabel('Voltage', fontsize=10)
            # plt.title(f'Voltage Trajectories - Aircraft {i+1}', fontsize=10)
            plt.grid(True, linestyle='--', alpha=0.7)

        # plt.tight_layout()  # Adjust subplot layout
            plt.tight_layout()
            filename = f'Battery_Voltage_trajectories_{i+1}.pdf'
            plt.savefig(os.path.join(self.current_directory, filename), format='pdf')
            # plt.savefig(os.path.join(self.current_directory, 'Battery_Voltage_trajectories_{i+1}.png'), format='png', dpi=140)
            # plt.show()
            plt.close()

    def plot_voltage_and_current(self):
        self.voltage_python = self.get_prog_voltage_forall_pickles()
        self.prog_times = self.get_prog_times()
        
        plt.rcParams["figure.figsize"] = (4, 4)  # Adjust figure size for multiple subplots
        wind_label = ['No', 'Medium', 'Low', 'Strong']

        for i, voltage_dict in enumerate(self.voltage_python):
            fig, ax1 = plt.subplots(figsize=(4, 4))  # Create a single subplot
            
            all_voltages = []
            min_length = float('inf')
            
            # Collect all voltages and find the minimum length
            for voltage_experiment in voltage_dict.values():
                for voltage in voltage_experiment:
                    all_voltages.append(voltage)
                    min_length = min(min_length, len(voltage))
            
            # Trim or pad all voltages to the minimum length
            trimmed_voltages = [voltage[:min_length] for voltage in all_voltages]
            
            time_stamp = len(self.prog_times[i]) - 3
            plot_time = self.prog_times[i][0:time_stamp]
            
            # Plot each voltage trajectory
            for voltage in trimmed_voltages:
                ax1.plot(plot_time, voltage[:time_stamp], linewidth=1, color='lightblue', alpha=0.8)

            # Compute the mean voltage trajectory
            mean_voltage = np.mean(trimmed_voltages, axis=0)
            ax1.plot(plot_time, mean_voltage[:time_stamp], linewidth=2, color='blue', label='Mean Voltage Trajectory')

            # Voltage subplot settings
            ax1.set_xlabel('Time (sec)', fontsize=10)
            ax1.set_ylabel('Voltage', fontsize=10)
            ax1.set_title(f'Voltage and Current Profiles - Aircraft {i+1}', fontsize=12)
            ax1.grid(True, linestyle='--', alpha=0.7)
            ax1.legend(prop={'size': 8}, loc='upper right')  # Smaller legend font size

            # Create a twin y-axis to plot the current profile
            ax2 = ax1.twinx()
            
            # Load current profile from .mat file
            results_directory = os.path.join(current_directory, "..", "EnergyRequirementResults/Journal_main1_wind0_energy0_path0_deviation0.9fullMissionBatteryParams.mat")
            mat_data = loadmat(results_directory)
            current_profile = mat_data['results'][0][1]  # Adjust this key based on your .mat file structure

            if True:
                # Plot current profile
                ax2.plot(plot_time, current_profile[:time_stamp], linewidth=2, color='green', label='Current Profile')
                ax2.set_ylabel('Current', fontsize=10)

                # Combine legends
                lines, labels = ax1.get_legend_handles_labels()
                lines2, labels2 = ax2.get_legend_handles_labels()
                ax1.legend(lines + lines2, labels + labels2, prop={'size': 8}, loc='upper right')
            

            plt.tight_layout()
            filename = f'Battery_Voltage_and_Current_Profile_{i+1}.png'
            plt.savefig(os.path.join(self.current_directory, filename), format='png')
            plt.close()
    
    
if __name__ == "__main__":
    current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
    # for experiment in range(0,6):
    results_directory = os.path.join(current_directory, "..", "EnergyRequirementResults/Journal_main1_wind0.2_energy0_path0_deviation0.9fullMissionBatteryParams.mat")
    pickle_path = os.path.join(current_directory, "..", "BatteryPrognosticsResults/Pickles/")
    plotter = PlotsPaper(pickle_path)
    # plotter.plot_soc_predictions_save_independetly()
    # plotter.plot_soc_predictions()
    plotter.plot_voltage_independently()
    plotter.plot_voltage_and_current()
    