import json
import matplotlib.pyplot as plt
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import griddata
from scipy.optimize import curve_fit

# Enable LaTeX in Matplotlib
plt.rcParams['text.usetex'] = True

def load_and_print_inner_keys(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            if isinstance(data, dict):
                for key in ['order', 'results']:
                    if key in data and isinstance(data[key], dict):
                        print(f"Keys inside '{key}': {data[key].keys()}")
                    else:
                        print(f"'{key}' is not present or not a dictionary.")
            else:
                print("JSON is not a dictionary.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")



def plot_order_points(file_path):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            order_data = data.get('order')

            if order_data and all(len(point) == 3 for point in order_data):
                longitudes = [point[0] for point in order_data]
                latitudes = [point[1] for point in order_data]
                altitudes = [point[2] for point in order_data]

                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')
                ax.scatter(longitudes, latitudes, altitudes)

                ax.set_xlabel('Longitude')
                ax.set_ylabel('Latitude')
                ax.set_zlabel('Altitude')

                plt.show()
            else:
                print("'order' key is not present or is not an Nx3 array.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")



def plot_wind_data(file_path, subsample_rate=2, vector_scale=120):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            order_data = data.get('order')
            wind_speeds = data.get('results', {}).get('wind_speeds')
            wind_directions = data.get('results', {}).get('wind_directions')

            if order_data and wind_speeds and wind_directions and \
               len(order_data) == len(wind_speeds) == len(wind_directions):
                # Subsample the data
                subsampled_indices = np.arange(0, len(order_data), subsample_rate)
                longitudes = np.array([point[0] for point in order_data])[subsampled_indices]
                latitudes = np.array([point[1] for point in order_data])[subsampled_indices]
                U = np.array([speed * np.cos(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])[subsampled_indices]
                V = np.array([speed * np.sin(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])[subsampled_indices]

                # Normalize the vectors
                vector_magnitudes = np.sqrt(U**2 + V**2)
                U_normalized = U / vector_magnitudes
                V_normalized = V / vector_magnitudes

                plt.figure(figsize=(15, 10))
                quiver = plt.quiver(longitudes, latitudes, U_normalized, V_normalized, vector_magnitudes, angles='xy', scale_units='xy', scale=vector_scale, cmap=plt.cm.jet, width=0.002)
                plt.colorbar(quiver, label=r'Wind Speed ($\frac{m}{s}$)')

                plt.title(r'\textbf{Normalized Wind Vectors}')
                plt.xlabel('Longitude')
                plt.ylabel('Latitude')
                plt.grid(True)
                plt.show()
            else:
                print("Data is missing or not in the expected format.")
    except FileNotFoundError:
        print("File not found.")
    except JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")
        
# def plot_wind_field_with_contour(file_path, subsample_rate=3, vector_scale=60):
def plot_wind_field_with_contour_and_trajectory(file_path, waypoint1, waypoint2, trajectory_lats, trajectory_lons, subsample_rate=3, vector_scale=60):
    
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            order_data = data.get('order')
            wind_speeds = data.get('results', {}).get('wind_speeds')
            wind_directions = data.get('results', {}).get('wind_directions')

            if order_data and wind_speeds and wind_directions and \
               len(order_data) == len(wind_speeds) == len(wind_directions):
                longitudes =  np.radians(np.array([point[0] for point in order_data]))
                latitudes =  np.radians(np.array([point[1] for point in order_data]))
                U = np.array([speed * np.cos(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
                V = np.array([speed * np.sin(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])

                magnitude = np.sqrt(U**2 + V**2)
                U_norm = U / magnitude
                V_norm = V / magnitude

                # Create a grid for the contour plot
                grid_x, grid_y = np.mgrid[min(longitudes):max(longitudes):100j, min(latitudes):max(latitudes):100j]
                grid_speed = griddata((longitudes, latitudes), magnitude, (grid_x, grid_y), method='linear')

                plt.figure(figsize=(15, 10))
                plt.contourf(grid_x, grid_y, grid_speed, cmap=plt.cm.jet)
                plt.colorbar(label='Wind Speed (magnitude)')
                
                indices = np.arange(0, len(longitudes), subsample_rate)
                plt.quiver(longitudes[indices], latitudes[indices], U_norm[indices], V_norm[indices], color='black', scale=vector_scale,width=0.002)
                # Plot the trajectory
                plt.plot(trajectory_lons, trajectory_lats, 'r-', linewidth=2, label='Trajectory')                
                # Mark the initial and final waypoints
                plt.scatter([waypoint1[1], waypoint2[1]], [waypoint1[0], waypoint2[0]], color='blue', marker='o', s=100, zorder=5, label='Waypoints')

                plt.title('Wind Field: Direction and Magnitude')
                plt.xlabel('Longitude')
                plt.ylabel('Latitude')
                plt.legend()
                plt.grid(True)
                plt.show()
            else:
                print("Data is missing or not in the expected format.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:  # Corrected the exception handling
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")
        
# Assuming the generate_trajectories_from_json function's output is passed as the 'trajectories' parameter
def plot_wind_field_with_contour_and_trajectories(file_path, subsample_rate=3, vector_scale=60):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
            order_data = data.get('order')
            wind_speeds = data.get('results', {}).get('wind_speeds')
            wind_directions = data.get('results', {}).get('wind_directions')

            if order_data and wind_speeds and wind_directions and \
               len(order_data) == len(wind_speeds) == len(wind_directions):
                longitudes = np.radians(np.array([point[0] for point in order_data]))
                latitudes = np.radians(np.array([point[1] for point in order_data]))
                U = np.array([speed * np.cos(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
                V = np.array([speed * np.sin(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])

                magnitude = np.sqrt(U**2 + V**2)
                U_norm = U / magnitude
                V_norm = V / magnitude

                # Create a grid for the contour plot
                grid_x, grid_y = np.mgrid[min(longitudes):max(longitudes):100j, min(latitudes):max(latitudes):100j]
                grid_speed = griddata((longitudes, latitudes), magnitude, (grid_x, grid_y), method='linear')

                plt.figure(figsize=(15, 10))
                plt.contourf(grid_x, grid_y, grid_speed, cmap=plt.cm.jet)
                plt.colorbar(label='Wind Speed (magnitude)')

                indices = np.arange(0, len(longitudes), subsample_rate)
                plt.quiver(longitudes[indices], latitudes[indices], U_norm[indices], V_norm[indices], color='black', scale=vector_scale, width=0.002)

                # Plot each trajectory
                # for plan_name, segments in trajectories.items():
                #     for segment in segments:
                #         trajectory_lats, trajectory_lons = segment
                #         plt.plot(np.radians(trajectory_lons), np.radians(trajectory_lats), '-', linewidth=2.3,  color='blue', label=f'Trajectory {plan_name}')
                #         # Mark the initial point
                #         plt.scatter(np.radians(trajectory_lons[0]), np.radians(trajectory_lats[0]), color='green', marker='o', s=100, zorder=5)
                #         # Mark the final point
                #         plt.scatter(np.radians(trajectory_lons[-1]), np.radians(trajectory_lats[-1]), color='red', marker='x', s=100, zorder=5)


                plt.title(r'\textbf{Wind Field: Direction and Magnitude with Trajectories}', fontsize=14)
                plt.xlabel('Longitude (radians)', fontsize=12)
                plt.ylabel('Latitude (radians)', fontsize=12)
                plt.legend()
                plt.grid(True)
                plt.savefig('wind_contour.png', format='png', bbox_inches='tight', dpi=300)
                plt.show()
            else:
                print("Data is missing or not in the expected format.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")

def polynomial_2d(x, y, *coeffs):
    """
    A general 2D polynomial function for curve fitting.
    x, y : Independent variables (latitude and longitude).
    coeffs : Coefficients of the polynomial terms.
    """
    # Assuming a simple 2D polynomial model for demonstration:
    # z = c + b1*x + b2*y + a11*x^2 + a12*x*y + a22*y^2
    # Adjust based on the actual degrees and terms of your polynomial model
    c, b1, b2, a11, a12, a22 = coeffs
    return c + b1*x + b2*y + a11*x**2 + a12*x*y + a22*y**2

# When calling curve_fit, ensure x_data and y_data are passed in correctly
# Now using a lambda function to correctly pass x and y as separate arguments to polynomial_2d
def fit_2d_polynomial_to_data(trajectories, wind_data):
    fitted_results = {}
    for plan_name, segments in trajectories.items():
        fitted_segments = []
        for segment_index, (lat_lon) in enumerate(segments):
            latitudes, longitudes = lat_lon
            speeds, directions = wind_data[plan_name][segment_index]

            # Flatten the data for curve fitting
            x_data = np.radians(np.array(latitudes))
            y_data = np.radians(np.array(longitudes))
            speeds = np.array(speeds)
            directions = np.array(directions)

            # Define initial guess for the polynomial coefficients
            initial_guess = [1, 1, 1, 1, 1, 1]  # Adjust as needed

            # Fit the model using curve_fit with a lambda function to unpack x and y
            # Fit the model using curve_fit with a lambda function to correctly pass x, y, and coefficients
            speed_coeffs, _ = curve_fit(lambda xy, *coeffs: polynomial_2d(xy[0], xy[1], *coeffs),
                                        (x_data, y_data), speeds, p0=initial_guess)
            direction_coeffs, _ = curve_fit(lambda xy, *coeffs: polynomial_2d(xy[0], xy[1], *coeffs),
                                            (x_data, y_data), directions, p0=initial_guess)


            fitted_segments.append((speed_coeffs, direction_coeffs))
        fitted_results[plan_name] = fitted_segments

    return fitted_results

        
def generate_trajectory(wpt1_rad, wpt2_rad, num_points=1000):
    """
    Generates a linear trajectory between two waypoints.

    Parameters:
    waypoint1 (list): The starting waypoint [latitude, longitude].
    waypoint2 (list): The ending waypoint [latitude, longitude].
    num_points (int): The number of points to generate in the trajectory.

    Returns:
    tuple: Two arrays containing latitudes and longitudes of the trajectory.
    """
    # Convert waypoints from degrees to radians
    # wpt1_rad = np.radians(waypoint1)
    # wpt2_rad = np.radians(waypoint2)
    
    trajectory_lats = np.linspace(wpt1_rad[0], wpt2_rad[0], num_points)
    trajectory_lons = np.linspace(wpt1_rad[1], wpt2_rad[1], num_points)
    return trajectory_lats, trajectory_lons

def generate_trajectories_from_json(flight_plans, num_points=1000):
    """
    Generates trajectories for each flight plan in the dictionary.

    Parameters:
    flight_plans (dict): Dictionary containing flight plans with waypoints.
    num_points (int): The number of points to generate for each trajectory segment.

    Returns:
    dict: A dictionary with the same keys as flight_plans, but each key contains a list of tuples for the latitudes and longitudes of the trajectory.
    """
    trajectories = {}
    for plan_name, waypoints in flight_plans.items():
        plan_trajectories = []
        for i in range(len(waypoints) - 1):
            trajectory_lats, trajectory_lons = generate_trajectory(waypoints[i], waypoints[i+1], num_points)
            plan_trajectories.append((trajectory_lats, trajectory_lons))
        trajectories[plan_name] = plan_trajectories
    return trajectories

        
def interpolate_along_trajectories(json_file, trajectories):
    with open(json_file, 'r') as file:
        data = json.load(file)
        order_data = data['order']
        wind_speeds = data['results']['wind_speeds']
        wind_directions = data['results']['wind_directions']

        # Extract longitudes and latitudes
        longitudes = np.radians(np.array([point[0] for point in order_data]))
        latitudes = np.radians(np.array([point[1] for point in order_data]))
        interpolated_results = {}

        for plan_name, segments in trajectories.items():
            segment_results = []
            for segment in segments:
                trajectory_lats, trajectory_lons = segment

                trajectory_lats_rad = np.radians(trajectory_lats)
                trajectory_lons_rad = np.radians(trajectory_lons)

                interpolated_speeds = griddata((longitudes, latitudes), wind_speeds, (trajectory_lons_rad, trajectory_lats_rad), method='linear')
                interpolated_directions = griddata((longitudes, latitudes), wind_directions, (trajectory_lons_rad, trajectory_lats_rad), method='linear')                

                segment_results.append((interpolated_speeds, interpolated_directions))
            interpolated_results[plan_name] = segment_results

        return interpolated_results

    
def fit_wind_data(longitudes, latitudes, wind_speeds, wind_directions):
    # Define a polynomial function to fit
    def poly_func(coords, a, b, c, d):
        x, y = coords
        return a * x + b * y + c * x * y + d

    # Clean the data: remove points where either speed or direction is NaN
    valid_indices = ~(np.isnan(wind_speeds) | np.isnan(wind_directions))
    longitudes_clean = longitudes[valid_indices]
    latitudes_clean = latitudes[valid_indices]
    wind_speeds_clean = wind_speeds[valid_indices]
    wind_directions_clean = wind_directions[valid_indices]

    # Fit for wind speeds
    params_speed, _ = curve_fit(poly_func, (longitudes_clean, latitudes_clean), wind_speeds_clean)
    # print(longitudes_clean)

    # Fit for wind directions
    params_direction, _ = curve_fit(poly_func, (longitudes_clean, latitudes_clean), wind_directions_clean)

    return params_speed, params_direction

def plot_interpolated_wind(trajectory_lats, trajectory_lons, wind_speeds, wind_directions, num_arrows=10):
    """
    Plots the trajectory with wind speed and direction.

    Parameters:
    trajectory_lats (numpy.ndarray): Array of latitudes in the trajectory.
    trajectory_lons (numpy.ndarray): Array of longitudes in the trajectory.
    wind_speeds (numpy.ndarray): Array of interpolated wind speeds along the trajectory.
    wind_directions (numpy.ndarray): Array of interpolated wind directions along the trajectory.
    num_arrows (int): Number of arrows to represent wind direction along the trajectory.
    """

    # Normalize wind speeds for color mapping
    norm = plt.Normalize(wind_speeds.min(), wind_speeds.max())

    plt.figure(figsize=(12, 6))

    # Plot trajectory with color gradient
    plt.scatter(trajectory_lons, trajectory_lats, c=wind_speeds, cmap='viridis', s=10)
    plt.colorbar(label='Wind Speed (m/s)')
    
    # Add arrows to indicate wind direction
    step = len(trajectory_lats) // num_arrows
    for i in range(0, len(trajectory_lats), step):
        end_lat = trajectory_lats[i] + 0.1 * np.sin(np.deg2rad(wind_directions[i]))  # Adjust the 0.01 if necessary
        end_lon = trajectory_lons[i] + 0.1 * np.cos(np.deg2rad(wind_directions[i]))  # Adjust the 0.01 if necessary
        # plt.arrow(trajectory_lons[i], trajectory_lats[i], end_lon - trajectory_lons[i], end_lat - trajectory_lats[i], head_width=0.005, head_length=0.001, fc='red', ec='red')

    plt.title('Interpolated Wind Data Along Trajectory')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.grid(True)
    plt.show()
    
def plot_comparison(trajectory_lons, trajectory_lats,  
                    interpolated_speeds, interpolated_directions,
                    fit_params_speed, fit_params_direction):
    # Calculate wind speeds and directions using fitted parameters
    fitted_speeds = fit_params_speed[0]*trajectory_lons + fit_params_speed[1]*trajectory_lats + fit_params_speed[2]*trajectory_lats*trajectory_lons + fit_params_speed[3]
    fitted_directions = fit_params_direction[0]*trajectory_lons + fit_params_direction[1]*trajectory_lats + fit_params_direction[2]*trajectory_lats*trajectory_lons + fit_params_direction[3]

    # Create subplots
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    # Plot wind speeds
    axs[0].plot(trajectory_lats, interpolated_speeds, 'b-', label='Interpolated Speeds')
    axs[0].plot(trajectory_lats, fitted_speeds, 'r--', label='Fitted Speeds')
    axs[0].set_ylabel('Wind Speed (m/s)')
    axs[0].legend()
    axs[0].set_title('Wind Speed Comparison')

    # Plot wind directions
    axs[1].plot(trajectory_lats, interpolated_directions, 'g-', label='Interpolated Directions')
    axs[1].plot(trajectory_lats, fitted_directions, 'y--', label='Fitted Directions')
    axs[1].set_xlabel('Latitude')
    axs[1].set_ylabel('Wind Direction (degrees)')
    axs[1].legend()
    axs[1].set_title('Wind Direction Comparison')

    plt.tight_layout()
    plt.show()

def polynomial_2d_eval(x, y, coeffs):
    # Example for a quadratic polynomial
    # coeffs = [c, b1, b2, a11, a12, a22]
    return coeffs[0] + coeffs[1]*x + coeffs[2]*y + coeffs[3]*x**2 + coeffs[4]*x*y + coeffs[5]*y**2



def plot_fitted_and_spatial_data_line(fitted_results, interpolated_wind_data, trajectories, plan_name):
    if plan_name not in fitted_results or plan_name not in interpolated_wind_data or plan_name not in trajectories:
        print(f"Plan {plan_name} not found in provided data.")
        return

    plt.figure(figsize=(14, 6))

    for segment_index, ((speed_coeffs, direction_coeffs), segment) in enumerate(zip(fitted_results[plan_name], trajectories[plan_name])):
        latitudes, _ = segment  # Assuming plotting against latitudes
        original_speeds, original_directions = interpolated_wind_data[plan_name][segment_index]
        
        # Assuming latitudes are sorted; if not, sort them along with speeds and directions
        sorted_indices = np.argsort(latitudes)
        latitudes_sorted = np.radians(np.array(latitudes)[sorted_indices])
        
        # Sort original speeds and directions based on sorted latitudes
        original_speeds_sorted = np.array(original_speeds)[sorted_indices]
        original_directions_sorted = np.array(original_directions)[sorted_indices]
        
        # Evaluate the fitted functions (assuming polynomial_2d_eval can handle 1D arrays for latitudes)
        fitted_speeds = np.polyval(speed_coeffs, latitudes_sorted)
        fitted_directions = np.polyval(direction_coeffs, latitudes_sorted)

        # Plot wind speed
        plt.subplot(1, 2, 1)
        plt.plot(latitudes_sorted, original_speeds_sorted, 'o-', label=f'Original Speed Segment {segment_index + 1}')
        plt.plot(latitudes_sorted, fitted_speeds, 'r--', label=f'Fitted Speed Segment {segment_index + 1}')

        # Plot wind direction
        plt.subplot(1, 2, 2)
        plt.plot(latitudes_sorted, original_directions_sorted, 'o-', label=f'Original Direction Segment {segment_index + 1}')
        plt.plot(latitudes_sorted, fitted_directions, 'r--', label=f'Fitted Direction Segment {segment_index + 1}')

    plt.subplot(1, 2, 1)
    plt.title(f'Wind Speed for {plan_name}')
    plt.xlabel('Latitude')
    plt.ylabel('Wind Speed')
    plt.legend()

    plt.subplot(1, 2, 2)
    plt.title(f'Wind Direction for {plan_name}')
    plt.xlabel('Latitude')
    plt.ylabel('Wind Direction')
    plt.legend()

    plt.tight_layout()
    plt.show()


    
def plot_fitted_and_spatial_data(fitted_results, interpolated_wind_data, trajectories, plan_name):
    if plan_name not in fitted_results or plan_name not in interpolated_wind_data or plan_name not in trajectories:
        print(f"Plan {plan_name} not found in provided data.")
        return

    plt.figure(figsize=(14, 6))

    for segment_index, ((speed_coeffs, direction_coeffs), segment) in enumerate(zip(fitted_results[plan_name], trajectories[plan_name])):
        latitudes, longitudes = np.radians(segment)
        original_speeds, original_directions = interpolated_wind_data[plan_name][segment_index]

        # Create a meshgrid for evaluation
        lat_grid, lon_grid = np.meshgrid(np.linspace(min(latitudes), max(latitudes), 100),
                                         np.linspace(min(longitudes), max(longitudes), 100))
        
        # Flatten the grid for evaluation
        lat_flat, lon_flat = lat_grid.flatten(), lon_grid.flatten()

        # Evaluate the fitted functions
        fitted_speeds = polynomial_2d_eval(lat_flat, lon_flat, speed_coeffs).reshape(lat_grid.shape)
        fitted_directions = polynomial_2d_eval(lat_flat, lon_flat, direction_coeffs).reshape(lat_grid.shape)

        # Plot original and fitted wind speed
        plt.subplot(1, 2, 1)
        plt.scatter(latitudes, longitudes, c=original_speeds, cmap='viridis', label='Original Speeds')
        plt.contourf(lat_grid, lon_grid, fitted_speeds, levels=100, cmap='viridis', alpha=0.5)
        plt.colorbar(label='Wind Speed')
        plt.xlabel('Latitude')
        plt.ylabel('Longitude')
        plt.title(f'Wind Speed for {plan_name}')

        # Plot original and fitted wind direction
        plt.subplot(1, 2, 2)
        plt.scatter(latitudes, longitudes, c=original_directions, cmap='magma', label='Original Directions')
        plt.contourf(lat_grid, lon_grid, fitted_directions, levels=100, cmap='magma', alpha=0.5)
        plt.colorbar(label='Wind Direction')
        plt.xlabel('Latitude')
        plt.ylabel('Longitude')
        plt.title(f'Wind Direction for {plan_name}')

    plt.tight_layout()
    plt.show()

def save_fitted_results_to_json(fitted_results, file_path):
    """
    Saves the fitted polynomial coefficients to a JSON file.

    Parameters:
    fitted_results (dict): Dictionary containing the coefficients of the fitted polynomial functions.
    file_path (str): Path to the JSON file where the data will be saved.
    """
    # Convert numpy arrays to lists for JSON serialization
    json_ready_data = {plan: [[speed_coeffs.tolist(), direction_coeffs.tolist()] for speed_coeffs, direction_coeffs in segments]
                       for plan, segments in fitted_results.items()}

    try:
        with open(file_path, 'w') as json_file:
            json.dump(json_ready_data, json_file, indent=4)
        print(f"Fitted results successfully saved to {file_path}.")
    except Exception as e:
        print(f"Failed to save fitted results to JSON: {e}")



#flight_plan_json = 'NASA_flight_plans.json'
#with open(flight_plan_json, 'r') as file:
#    flight_plan = json.load(file)
    
trajectories = [] #generate_trajectories_from_json(flight_plan, num_points=1000)
# plt.plot(trajectories['Flight_plan_3'][0][0],trajectories['Flight_plan_3'][0][1])
# plt.show()
# print(trajectories['Flight_plan_3'][0][0])
wind_data = 'wind_data_DFW_at_0_5km_new.json'
#interpolated_wind_data = interpolate_along_trajectories(wind_data, trajectories)
plot_wind_field_with_contour_and_trajectories('wind_data_DFW_at_0_5km_new.json', subsample_rate=3, vector_scale=60)


