
import scipy.io as sio
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['text.usetex'] = True


# Function to load data from .mat files
def load_mat_file(file_path):
    return sio.loadmat(file_path)

# Function to extract experimental data
def extract_data(mat_data, experiment_index):
    SOC = mat_data['results'][experiment_index][0]
    totalCurrent = mat_data['results'][experiment_index][1]
    voltage = mat_data['results'][experiment_index][2]
    actualPosTraj = mat_data['results'][experiment_index][3]
    refPosTraj = mat_data['results'][experiment_index][4]
    time = mat_data['results'][experiment_index][5].flatten()
    timeb = mat_data['results'][experiment_index][6].flatten()
    return SOC, totalCurrent, voltage, actualPosTraj, refPosTraj, time, timeb

# Function to calculate power and cumulative energy with detailed shape validation
def compute_power_profile(voltage, current, time):
    # Convert to numpy arrays and check lengths
    voltage = np.array(voltage).flatten()
    current = np.array(current).flatten()
    time = np.array(time).flatten()

    # Calculate power
    power = voltage * current

    # Calculate cumulative energy with consistent shapes
    energy = np.trapz(power, time)/1000
    return power, energy



# Function to generate and save the plot
def plot_power_profiles(time_no_eff, power_no_eff, energy_no_eff, 
                        time_with_eff, power_with_eff, energy_with_eff):
    plt.figure(figsize=(3.5, 3.5))
    plt.plot(time_no_eff, power_no_eff, 'r--', label='Baseline', linewidth=1.5)
    plt.plot(time_with_eff, power_with_eff,'b-', label='Energy-Efficient', linewidth=1.5)
    # plt.title('Power Profile Comparison for Aircraft 1')
    plt.xlabel('Time (s)')
    plt.ylabel('Power (W)')
    plt.legend(loc='lower left')
    plt.grid(True, linestyle='--')

    # Add text for cumulative energy
    plt.text(0.1, 0.95, f'Energy (Baseline): {energy_no_eff:.2f} KJ', 
             transform=plt.gca().transAxes, fontsize=10, verticalalignment='top')
    plt.text(0.1, 0.9, f'Energy (Energy-Efficient): {energy_with_eff:.2f} KJ', 
             transform=plt.gca().transAxes, fontsize=10, verticalalignment='top')
    plt.tight_layout()

    # Save and display the plot
    plt.savefig('power_profile_comparison_aircraft_2.pdf', bbox_inches='tight')
    plt.show()

# Main function to execute the data processing and plotting
def main():

    # Load the .mat files
    mat_file_1 = sio.loadmat('Journal_main1_wind0_energy0_path0_deviation0fullMissionBatteryParams.mat')
    mat_file_2 = sio.loadmat('Journal_main1_wind0_energy0_path0_deviation1fullMissionBatteryParams.mat')

    # Extract data for aircraft 1 from both files
    SOC_1_no_eff, current_1_no_eff, voltage_1_no_eff, actualTraj_1_no_eff, refTraj_1_no_eff, time_1_no_eff, timeb_1_no_eff = extract_data(mat_file_1, 1)
    SOC_1_with_eff, current_1_with_eff, voltage_1_with_eff, actualTraj_1_with_eff, refTraj_1_with_eff, time_1_with_eff, timeb_1_with_eff = extract_data(mat_file_2, 1)

    # Calculate power and cumulative energy for both cases
    power_1_no_eff, energy_1_no_eff = compute_power_profile(voltage_1_no_eff, current_1_no_eff, timeb_1_no_eff)
    power_1_with_eff, energy_1_with_eff = compute_power_profile(voltage_1_with_eff, current_1_with_eff, timeb_1_with_eff)

    # Generate the plot
    plot_power_profiles(timeb_1_no_eff, power_1_no_eff, energy_1_no_eff, 
                        timeb_1_with_eff, power_1_with_eff, energy_1_with_eff)

# Run the main function
if __name__ == "__main__":
    main()
