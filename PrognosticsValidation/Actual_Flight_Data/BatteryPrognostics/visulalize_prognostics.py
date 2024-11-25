import os
import pickle 
import numpy as np
import matplotlib.pyplot as plt
import collections
import pandas as pd
from scipy import stats
from scipy.stats import norm
from prog_algs.metrics import prob_success
from sklearn.preprocessing import normalize 
from pathlib import Path
import sys 
plt.rcParams['text.usetex'] = True

class VisualizePrognostics:
    """
    A class for generating battery prognostics related graphics and plots.

    Attributes:
        prog_results (str): Path to the ".pkl" file containing prognostics outputs.
        num_traces (int): number of MC simulations for prognostics.
        prog_times (array): time stamps for prognostics simulations.
        voltage_python (array): battery voltage generated from prognostics process 
        voltage_matlab (array): battery voltage generated from UAV code (MATLAB)  
        probability_sucess (float): the probability of mission success computed from the prognostics experiment
        soc_threshold (float): SOC threshold value used to compute the probability of mission success
        timeb_matlab (array): time scale of the battery data generated from the UAV (MATLAB)
        overlay_plots (boolean): to/not to plot the battery data generated from matlab and python on one plot 

    Methods:
        load_pickle_file: loads the pickle file that contains all info related to prognostics experiment
        get_prog_voltage: parses voltage traces from the pickle file 
        get_SOC: parses SOC traces from the pickle file  
        plot_XXX: plots the parameter and saves it inside "Results/Plots/Experiment_i folder 
    """
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
        self.current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
        self.plots_directory = os.path.join(self.current_directory, "..", "BatteryPrognosticsResults/Pickles")
    
    def load_pickle_file(self, pickle_path):
        with open (pickle_path, 'rb') as pickle_file:
            mc_results = pickle.load(pickle_file)
        return mc_results
    
    def get_prog_voltage(self):
        # Note follow the same procedure if you want to get temprature in the future
        voltage = []
        for sim in range(0, len(self.prog_results.outputs)):
            voltage.append([])
            for time in range(0, len(self.prog_results.outputs[sim])):
                # print(self.prog_results.outputs[sim])
                voltage[sim].append(self.prog_results.outputs[sim][time]['v']) #<-- store voltage into an array
        return voltage
    
    def get_SOC(self):
        soc = []
        for i in range(0, len(self.prog_results.event_states)):
            soc.append([])
            for t in range(0, len(self.prog_times)-2):
                soc[i].append(self.prog_results.event_states[i][t]['EOD'])
        SOC = np.array(soc)
        return SOC
    
    def plot_soc_trajectories(self):
        # self.plots_directory = os.path.join(self.current_directory, "..", "Results/Plots/Experiment_{}".format(self.experiment_index))
        time_stamp = int(len(self.current_input)*self.sample_time)
        for trace in range(0,self.num_traces):
            plt.plot(self.soc_python[trace, 0:time_stamp], linewidth=1, color='black')
        plt.xlabel('time')
        plt.ylabel('SOC')
        plt.grid(True)
        plt.title('Battery SOC Curve')
        plt.show
        plt.savefig(self.plots_directory+'/SOC_curve.png')
        plt.close()
        
        
    def plot_soc_prediction(self):
        # self.plots_directory = os.path.join(self.current_directory, "..", "Results/Plots/Experiment_{}".format(self.experiment_index))
        plt.subplots_adjust(bottom=0.2)
        plt.rcParams["figure.figsize"] = (6,5)
        time_stamp = int(len(self.current_input)*self.sample_time)
        SOC = self.soc_python[:,time_stamp]
        mu = np.mean(SOC)
        sigma = np.std(SOC)
        # Generate the bins for the histogram
        fig, ax0 = plt.subplots(ncols=1, nrows=1) #creating plot axes
        (values, bins, _) = ax0.hist(SOC,bins=100,density=True,label="Histogram",linewidth=2)
        # Generate the pdf
        bin_centers = 0.5*(bins[1:] + bins[:-1])
        pdf = stats.norm.pdf(x = bin_centers, loc=mu, scale=sigma) #Compute probability density function
        ax0.plot(bin_centers, pdf, label="PDF",color='black',linewidth=2) #Plot PDF
        ax0.axvline(x=self.soc_threshold, color='red',label="SOC threshold",linewidth=2, linestyle='--')
        ax0.legend(prop={'size': 12})
        plt.xlabel('SOC at {time} sec'.format(time=time_stamp), fontsize=13)
        plt.ylabel('count', fontsize=13)
        plt.grid()
        plt.title(r'\textbf{Battery SOC prediction}', fontsize=14)
        plt.savefig(self.plots_directory+'/Battery_SOC_low.png', format='png', dpi=140)
        plt.close()
        # plt.savefig('Battery_SOC_medium.eps', format='eps')
        
    def compute_prob_success(self):
        time_stamp = int(len(self.current_input)*self.sample_time)
        SOC = self.soc_python[:,time_stamp]
        self.probability_sucess = sum(SOC > self.soc_threshold)/self.num_traces
        print('The probability of success for the mission is', self.probability_sucess)
        return self.probability_sucess
    
    def plot_voltage(self):
        # self.plots_directory = os.path.join(self.current_directory, "..", "Results/Plots/Experiment_{}".format(self.experiment_index))
        # plt.subplots_adjust(bottom=0.3)
        plt.rcParams["figure.figsize"] = (6,5)
        time_stamp = int(len(self.current_input)*self.sample_time)
        for voltage in self.voltage_python:
            plt.plot(self.prog_times[0:time_stamp],voltage[0:time_stamp],linewidth=1, color='blue')
            
        plt.plot(self.prog_times[0:time_stamp],voltage[0:time_stamp],linewidth=1, color='blue', label='Simulated voltage')
        # plt.plot(actual_voltage[0::10],linewidth=1, color='red', label='Actual voltage')    
        plt.xlabel('time (sec)', fontsize=13)
        plt.ylabel('voltage', fontsize=13)
        plt.title(r'\textbf{500 MC simulations (V)}', fontsize=14)
        plt.legend(prop={'size': 12})
        plt.grid()
        plt.savefig(self.plots_directory+'/MC_Sim_Voltage_strong.png', format='png', dpi=140)
        plt.close(not self.overlay_plots)
        # plt.savefig('MC_Sim_Voltage_strong.eps', format='eps')
    
    def compare_matlab_with_python(self):
        # self.plots_directory = os.path.join(self.current_directory, "..", "Results/Plots/Experiment_{}".format(self.experiment_index))
        plt.plot(self.timeb_matlab[0], self.voltage_matlab)
        plt.savefig(self.plots_directory+'/UAV_voltage.png', format='png', dpi=140)
        plt.close()
        
        plt.plot(self.timeb_matlab[0], self.current_input)
        plt.savefig(self.plots_directory+'/UAV_current.png', format='png', dpi=140)
        plt.close()
        
        plt.plot(self.timeb_matlab[0], self.SOC_matlab)
        plt.savefig(self.plots_directory+'/UAV_SOC.png', format='png', dpi=140)
        plt.close()
        pass  
        
        
        
        
          
            
        
        

