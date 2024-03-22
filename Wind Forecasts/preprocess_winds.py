import os
import json
import matplotlib.pyplot as plt
from scipy.io import loadmat
import matplotlib
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from scipy.interpolate import griddata
from scipy.optimize import curve_fit
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.pipeline import make_pipeline

# Enable LaTeX in Matplotlib
plt.rcParams['text.usetex'] = True


def calculate_wind_components(poly_model_u, poly_model_v, xs, ys, degree):
    # For poly_model_u
    # Access the named_steps attribute of the pipeline to get to the LinearRegression object
    coefs_u = poly_model_u.named_steps['linearregression'].coef_
    intercept_u = poly_model_u.named_steps['linearregression'].intercept_

    # For poly_model_v
    coefs_v = poly_model_v.named_steps['linearregression'].coef_
    intercept_v = poly_model_v.named_steps['linearregression'].intercept_
            

    xs = np.array(xs).reshape(-1, 1)
    ys = np.array(ys).reshape(-1, 1)
    features = np.hstack((xs, ys))
    poly_features = PolynomialFeatures(degree=degree, include_bias=True).fit_transform(features)
    
    # Calculate the wind components by dot product, then add the intercept
    Us = np.dot(poly_features, coefs_u) + intercept_u
    Vs = np.dot(poly_features, coefs_v) + intercept_v
    
    return Us, Vs


def meters_to_latlon(x, y, center_lat, center_lon):
    # Constants
    meters_per_degree_lat = 111319
    meters_per_degree_lon = meters_per_degree_lat * np.cos(np.radians(center_lat))

    # Convert
    delta_lat = y / meters_per_degree_lat
    delta_lon = x / meters_per_degree_lon

    return center_lat + delta_lat, center_lon + delta_lon

def lat_lon_to_meters(lat, lon, center_lat, center_lon):
    # Earth's radius in meters
    R = 6371000
    # Convert latitude and longitude differences to radians
    delta_lat = lat - center_lat
    delta_lon = lon - center_lon
    # Approximate conversion to meters
    m_lat = delta_lat * R
    m_lon = delta_lon * R * np.cos(center_lat)
    return m_lat, m_lon


def plot_uav_trajectories_on_wind_contour(directory_path, center_lat, center_lon):
    for file_name in os.listdir(directory_path):
        if file_name.endswith('.mat'):
            file_path = os.path.join(directory_path, file_name)
            data = loadmat(file_path)
            results = data['results']
            
            for uav in range(results.shape[0]):
                trajectory = results[uav, 3]  # 4th parameter contains the x, y, z trajectory
                
                # Convert from meters to geographic coordinates
                latitudes, longitudes = [], []
                for point in trajectory:
                    lat, lon = meters_to_latlon(point[0], point[1], center_lat, center_lon)
                    latitudes.append(lat)
                    longitudes.append(lon)
                
                latitudes = np.radians(latitudes)
                longitudes = np.radians(longitudes)
                
                # Plot UAV trajectory
                plt.plot(longitudes, latitudes, '-', linewidth=2.3, color='blue', label=f'UAV {uav+1} Trajectory')
                # Mark the initial point
                plt.scatter(longitudes[0], latitudes[0], color='green', marker='o', s=100, zorder=5)
                # Mark the final point
                plt.scatter(longitudes[-1], latitudes[-1], color='red', marker='x', s=100, zorder=5)

    plt.legend()
    plt.show()
        
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
                ### This section fits the curve and prints the parameters 
                # Convert latitudes and longitudes to meters
                center_lon = np.radians((-96.84 + -96.81) / 2)
                center_lat = np.radians((33.14 + 33.17) / 2) 
                m_lats, m_lons = lat_lon_to_meters(latitudes, longitudes, center_lat, center_lon)
                # Stack m_lons and m_lats to create a 2D array of features
                features = np.vstack((m_lons, m_lats)).T  # Shape should be [n_samples, n_features]

                # Define the degree of the polynomial fit
                degree = 2  # Example degree, can be adjusted based on the complexity of your data
                # Create a pipeline that creates polynomial features and then fits a linear regression model
                poly_model_u = make_pipeline(PolynomialFeatures(degree), LinearRegression())
                poly_model_v = make_pipeline(PolynomialFeatures(degree), LinearRegression())

                # Fit the models for U and V components
                poly_model_u.fit(features, U)
                poly_model_v.fit(features, V)
                # For poly_model_u
                # Access the named_steps attribute of the pipeline to get to the LinearRegression object
                coefs_u = poly_model_u.named_steps['linearregression'].coef_
                intercept_u = poly_model_u.named_steps['linearregression'].intercept_

                # For poly_model_v
                coefs_v = poly_model_v.named_steps['linearregression'].coef_
                intercept_v = poly_model_v.named_steps['linearregression'].intercept_
                print("Coefficients for poly_model_u:")
                print(coefs_u)
                print("Intercept for poly_model_u:")
                print(intercept_u)

                print("\nCoefficients for poly_model_v:")
                print(coefs_v)
                print("Intercept for poly_model_v:")
                print(intercept_v)               
              

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

                plt.title(r'\textbf{Wind Field: Direction and Magnitude with Trajectories}', fontsize=14)
                plt.xlabel('Longitude (radians)', fontsize=12)
                plt.ylabel('Latitude (radians)', fontsize=12)
                plt.legend()
                plt.grid(True)
                plt.savefig('wind_contour.png', format='png', bbox_inches='tight', dpi=300)
                # plt.show()
            else:
                print("Data is missing or not in the expected format.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return poly_model_u, poly_model_v

   
def plot_comparison_and_error(file_path, coefs_u, poly_model_u, coefs_v, poly_model_v, center_lat, center_lon):
    """
    Plots original and fitted wind field data, and calculates the error.
    
    Parameters:
    - original_data: Dict containing 'order_data', 'wind_speeds', 'wind_directions'
    - fitted_params_u: Parameters of the fitted function for the U component
    - fitted_params_v: Parameters of the fitted function for the V component
    - center_lat, center_lon: Reference point for converting lat/lon to meters
    """
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
                ### This section fits the curve and prints the parameters 
                # Convert latitudes and longitudes to meters
                center_lon = np.radians((-96.84 + -96.81) / 2)
                center_lat = np.radians((33.14 + 33.17) / 2) 
                m_lats, m_lons = lat_lon_to_meters(latitudes, longitudes, center_lat, center_lon)

    
    # Calculate original U and V components
    U_orig = np.array([speed * np.cos(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
    V_orig = np.array([speed * np.sin(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
    
    # Predict U and V components using the fitted parameters
    lat_m = np.array(m_lats).reshape(-1, 1)
    long_m = np.array(m_lons).reshape(-1, 1)
    
    degree = 2
    [U_pred,V_pred] = calculate_wind_components(poly_model_u, poly_model_v, long_m, lat_m, degree)
    # print((lat_m))
 

    # Combine lat_m and long_m into a single array of shape (n_samples, n_features)
    features = np.hstack((long_m, lat_m))

    # Use the fitted models to predict the U and V components
    # Calculate error
    error_u = np.mean(np.abs(U_orig - U_pred))
    error_v = np.mean(np.abs(V_orig - V_pred))
        # Calculate magnitudes for original and predicted
    magnitude_orig = np.sqrt(U_orig**2 + V_orig**2)
    magnitude_pred = np.sqrt(U_pred**2 + V_pred**2)
    
    # Create a mesh grid for contour plots
    grid_x, grid_y = np.mgrid[min(longitudes):max(longitudes):100j, min(latitudes):max(latitudes):100j]
    
    # Interpolate magnitudes onto the grid
    grid_z_orig = griddata((longitudes, latitudes), magnitude_orig, (grid_x, grid_y), method='linear')
    grid_z_pred = griddata((longitudes, latitudes), magnitude_pred, (grid_x, grid_y), method='linear')

    # Plotting
    fig, axes = plt.subplots(1, 2, figsize=(20, 10), sharey=True)
    
    # Original wind field with contours
    contour_orig = axes[0].contourf(grid_x, grid_y, grid_z_orig, cmap='viridis', levels=15, alpha=0.5)
    fig.colorbar(contour_orig, ax=axes[0], label='Wind Speed (magnitude)')
    axes[0].quiver(longitudes, latitudes, U_orig/magnitude_orig, V_orig/magnitude_orig, color='r', scale=50)
    axes[0].set_title('Original Wind Field')
    
    # Predicted wind field with contours
    contour_pred = axes[1].contourf(grid_x, grid_y, grid_z_pred, cmap='viridis', levels=15, alpha=0.5)
    fig.colorbar(contour_pred, ax=axes[1], label='Wind Speed (magnitude)')
    axes[1].quiver(longitudes, latitudes, U_pred/magnitude_pred, V_pred/magnitude_pred, color='b', scale=50)
    axes[1].set_title('Predicted Wind Field')
    
    for ax in axes:
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.grid(True)
    
    # plt.show()


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


wind_data = 'wind_data_frisco_100m_time2.json'
[poly_model_u, poly_model_v] = plot_wind_field_with_contour_and_trajectories('wind_data_frisco_100m_time2.json', subsample_rate=3, vector_scale=60)
directory_path = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults'
# Define the center coordinates
center_lon = (-96.84 + -96.81) / 2
center_lat = (33.14 + 33.17) / 2 

coefs_u = [ 0.00000000e+00,  5.34005812e-05, -9.30620012e-05,  1.63456334e-08,
 -3.91258787e-09,  4.90305708e-09]
intercept_u = -14.039865205417248
coefs_v = [ 0.00000000e+00, -5.18105824e-06, -2.74580632e-05, -1.35881785e-08,
 -1.00454790e-08, -2.57029039e-09]
intercept_v = -5.604957270921905
plot_comparison_and_error(wind_data, coefs_u, poly_model_u, coefs_v, poly_model_v, center_lat, center_lon)
plot_uav_trajectories_on_wind_contour(directory_path, center_lat, center_lon)
   