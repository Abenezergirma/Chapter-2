import os
import scipy.io
import numpy as  np
from BatteryPrognosticsLibrary import BatteryPrognostics
from visualize_validation import VisualizePrognostics


class BatteryExperiment:
    """
    A class for conducting battery prognostics experiments based on MATLAB/Simulink simulation outputs.

    This class takes a ".mat" file containing UAV simulation results as input. The simulation results are
    parsed and fed into the BatteryPrognostics class from BatteryPrognosticsLibrary.py to perform prognostics
    analysis on the battery. The resulting prognostics outputs are saved as pickle files in the
    "Results/Prognostics/Outputs" folder.

    Attributes:
        result_file (str): Path to the ".mat" file containing simulation outputs.
        num_simulations (int): number of MC simulations for prognostics.
        experiment_index (int): index to show which experiment is currently running.
        VEOD (float): battery voltage threshold to assign end of discharge (EOD).
        predictor_method (str): prediction approach used to perform prognostics.
        data (.mat): the data generated from UAV simulation.
        battery_prog (class): a class that does prognostics
        visualize_prog (class): a class that generated the prognostics plots 

    Methods:
        print_properties_and_confirm: prints the experiment variables and confirm with the user.
        run: runs the entire battery experiment.
    """
    def __init__(self, result_file, pickle_path, experiment_index):
        """
        Initialize the BatteryPrognosticsExperiment instance.

        Args:
            result_file (str): Path to the ".mat" file containing simulation outputs.
            pickle_path (str): Path to the directory where the prognostics outputs will be saved.
            experiment_index (int): index to show which experiment is currently running.
        """
        self.result_file = result_file
        self.num_simulations = 500
        self.experiment_index = experiment_index 
        self.VEOD = 18
        self.predictor_method = "MC"
        self.data = scipy.io.loadmat(self.result_file)
        
        self.aircraftID = experiment_index
        # print(self.data['results'][self.aircraftID][1])
        self.SOC = self.data['SOC']
        self.totalCurrent = self.data['totalCurrent']
        # print(self.totalCurrent)
        self.voltage = self.data['voltage']
        self.actualPosTraj = self.data['llaTraj']
        self.time = self.data['time']
        self.timeb = self.data['time']#[self.aircraftID][6]
        self.sample_time = 0.1
        self.battery_prog = BatteryPrognostics(
            self.totalCurrent,self.num_simulations,self.sample_time,self.VEOD,self.predictor_method
        )
        self.battery_prog.experiment_index = self.experiment_index
        
        self.current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
        self.pickel_directory = os.path.join(self.current_directory, "..", "ValidationResults/Pickles")
        
    
    def print_properties_and_confirm(self):
        """
        Prints the experiment variables and confirm with the user.
        """
        print("\nProperty Values:")
        print(f"{'Property':<25} {'Value':<15}")
        print("-" * 30)
        print(f"{'current_imput':<25} {self.result_file:<15}")
        print(f"{'No of simulations':<25} {self.num_simulations:<15}")
        print(f"{'Sample time':<25} {self.sample_time:<15}")
        print(f"{'VEOD':<25} {self.VEOD:<15}")
        print(f"{'Predictor Method':<25} {self.predictor_method:<15}")
        print("-" * 30)
        while True:
            confirmation = input("Are the values correct? (yes/no): ").lower()
            if confirmation == "yes":
                print("Proceeding to the next steps...")
                break
            elif confirmation == "no":
                print("Values are not valid. Please update them.")
                self.result_file = input("Enter path to current input: ")
                self.num_simulations = int(input("Enter desired number of simulations: "))
                self.sample_time = input("Enter new sample time: ")
                self.VEOD = input("Enter new voltage threshold: ")
                self.predictor_method = input("Enter predictor method: ")
                self.print_properties_and_confirm()
                break
            else:
                print("Invalid input. Please enter 'yes' or 'no'.")
              

    def run(self):
        """
        Runs battery prognostics using the data generated from UAV model (MATLAB) 
        """
        # self.print_properties_and_confirm()
        # self.battery_prog.main()
        # Visualize the prognostics experiment
        self.visualize_prog = VisualizePrognostics(pickle_path)
        self.visualize_prog.current_input = self.totalCurrent
        self.visualize_prog.sample_time = self.sample_time
        self.visualize_prog.num_traces = self.num_simulations
        self.visualize_prog.voltage_matlab = self.voltage
        self.visualize_prog.SOC_matlab = self.SOC
        self.visualize_prog.timeb_matlab = self.timeb
        self.visualize_prog.experiment_index = self.experiment_index
        self.visualize_prog.llaTraj = self.actualPosTraj
        self.visualize_prog.plot_3d_trajectory()
        self.visualize_prog.plot_current_profile()
        self.visualize_prog.plot_voltage_comparison()
        self.visualize_prog.plot_soc_comparison()
        self.visualize_prog.save_side_by_side_plots()
        self.visualize_prog.compute_prognostics_accuracy()


if __name__ == "__main__":
    current_directory = os.path.dirname(os.path.abspath(__file__))  # Get the current directory of the script
    for experiment in range(0,1):
        results_directory = os.path.join(current_directory, "Actual_Flight_Data/LongMissionExtracted.mat")
        pickle_path = os.path.join(current_directory,  "ValidationResults/Pickles/prog_results_{}.pkl".format(experiment))
        experiment = BatteryExperiment(results_directory,pickle_path, experiment)
        experiment.run()
