import os
import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import loadmat
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
 

def generate_functional_wind_data_for_area(lower_left, upper_right, num_points=1000):
    """
    Generates wind data for an area defined by diagonal coordinates using functional relationships.
    
    Args:
    - lower_left: Tuple of (longitude, latitude) for the lower left corner.
    - upper_right: Tuple of (longitude, latitude) for the upper right corner.
    - num_points: Number of data points to generate.
    
    Returns:
    - List of wind data points with coordinates and wind properties based on a functional relationship.
    """
    lons = np.linspace(lower_left[0], upper_right[0], int(np.sqrt(num_points)))
    lats = np.linspace(lower_left[1], upper_right[1], int(np.sqrt(num_points)))
    
    wind_data = []
    speed_range = [5,11]
    direction_range = [0, 250]
    min_speed, max_speed = speed_range
    min_direction, max_direction = direction_range
    for lon in lons:
        for lat in lats:
            # Ensure a more uniform distribution across the specified range
            norm_factor = np.pi / (upper_right[0] - lower_left[0])
            speed_factor = (np.sin(norm_factor * lon) + 1) / 2  # Normalized to [0, 1]
            direction_factor = (np.cos(norm_factor * lat) + 1) / 2  # Normalized to [0, 1]

            wind_speed = min_speed + (max_speed - min_speed) * speed_factor
            wind_direction = min_direction + (max_direction - min_direction) * direction_factor
             
            wind_data.append({
                "coordinates": [lon, lat],
                "wind_speed": wind_speed,
                "wind_direction": wind_direction
            })
    
    return wind_data

def plot_wind_field_with_contour(file_path, vector_scale=60, subsample_rate=3):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        
        wind_data = data.get('wind_data', [])
        if wind_data:
            # Extract longitude, latitude, wind speed, and wind direction from the data
            longitudes = np.radians(np.array([point['coordinates'][0] for point in wind_data]))
            latitudes = np.radians(np.array([point['coordinates'][1] for point in wind_data]))
            wind_speeds = np.array([point['wind_speed'] for point in wind_data])
            wind_directions = np.array([point['wind_direction'] for point in wind_data])

            # Convert wind direction and speed into U and V components
            U = wind_speeds * np.cos(np.deg2rad(wind_directions))
            V = wind_speeds * np.sin(np.deg2rad(wind_directions))
            ### This section fits the curve and prints the parameters 
            # Convert latitudes and longitudes to meters
            center_lon = np.radians((-96.84 + -96.81) / 2)
            center_lat = np.radians((33.14 + 33.17) / 2) 
            m_lats, m_lons = lat_lon_to_meters(latitudes, longitudes, center_lat, center_lon)
            # Stack m_lons and m_lats to create a 2D array of features
            features = np.vstack((m_lons, m_lats)).T  # Shape should be [n_samples, n_features]

            # Define the degree of the polynomial fit
            degree = 4  
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

            # Normalize the wind vectors for uniform arrow size
            magnitude = np.sqrt(U**2 + V**2)
            U_norm = U / magnitude
            V_norm = V / magnitude

            # Create a grid for the contour plot
            grid_x, grid_y = np.mgrid[min(longitudes):max(longitudes):100j, min(latitudes):max(latitudes):100j]
            print(min(longitudes),max(longitudes))
            print(min(latitudes),max(latitudes))

            
            grid_speed = griddata((longitudes, latitudes), magnitude, (grid_x, grid_y), method='linear')

            plt.figure(figsize=(10, 7))
            plt.contourf(grid_x, grid_y, grid_speed, cmap=plt.cm.jet)
            plt.colorbar(label='Wind Speed (magnitude)')

            indices = np.arange(0, len(longitudes), subsample_rate)
            plt.quiver(longitudes[indices], latitudes[indices], U_norm[indices], V_norm[indices], color='black', scale=vector_scale, width=0.002)

            plt.title('Wind Field: Direction and Magnitude', fontsize=14)
            plt.xlabel('Longitude', fontsize=12)
            plt.ylabel('Latitude', fontsize=12)
            plt.grid(True)
            plt.savefig('wind_contour.png', format='png', bbox_inches='tight', dpi=300)
            plt.show()
        else:
            print("Wind data is missing or not in the expected format.")
    except FileNotFoundError:
        print("File not found.")
    except json.JSONDecodeError:
        print("Error decoding JSON.")
    except Exception as e:
        print(f"An error occurred: {e}")
    return poly_model_u, poly_model_v


def plot_comparison_and_error(file_path, poly_model_u, poly_model_v, center_lat, center_lon):
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
        
    wind_data = data.get('wind_data', [])
    if wind_data:
        # Extract longitude, latitude, wind speed, and wind direction from the data
        longitudes = np.radians(np.array([point['coordinates'][0] for point in wind_data]))
        latitudes = np.radians(np.array([point['coordinates'][1] for point in wind_data]))
        wind_speeds = np.array([point['wind_speed'] for point in wind_data])
        wind_directions = np.array([point['wind_direction'] for point in wind_data])

        # Convert wind direction and speed into U and V components
        U = wind_speeds * np.cos(np.deg2rad(wind_directions))
        V = wind_speeds * np.sin(np.deg2rad(wind_directions))
       ### This section fits the curve and prints the parameters 
        # Convert latitudes and longitudes to meters
        center_lon = np.radians((-96.84 + -96.81) / 2)
        center_lat = np.radians((33.14 + 33.17) / 2) 
        m_lats, m_lons = lat_lon_to_meters(latitudes, longitudes, center_lat, center_lon)

    
    # Calculate original U and V components
    U_orig = np.array([speed * np.cos(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
    V_orig = np.array([speed * np.sin(np.deg2rad(direction)) for speed, direction in zip(wind_speeds, wind_directions)])
    
    lat_m = np.array(m_lats).reshape(-1, 1)
    long_m = np.array(m_lons).reshape(-1, 1)
    
    degree = 4
    [U_pred,V_pred] = calculate_wind_components(poly_model_u, poly_model_v, long_m, lat_m, degree)
 
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
    subsample_rate = 2
    indices = np.arange(0, len(longitudes), subsample_rate)
    
    # Original wind field with contours
    contour_orig = axes[0].contourf(grid_x, grid_y, grid_z_orig, cmap='viridis', levels=15, alpha=0.5)
    fig.colorbar(contour_orig, ax=axes[0], label='Wind Speed (magnitude)')
    axes[0].quiver(longitudes[indices], latitudes[indices], U_orig[indices]/magnitude_orig[indices], V_orig[indices]/magnitude_orig[indices], color='r', scale=50)
    axes[0].set_title('Original Wind Field')
    
    
    # Predicted wind field with contours
    contour_pred = axes[1].contourf(grid_x, grid_y, grid_z_pred, cmap='viridis', levels=15, alpha=0.5)
    fig.colorbar(contour_pred, ax=axes[1], label='Wind Speed (magnitude)')
    axes[1].quiver(longitudes[indices], latitudes[indices], U_pred[indices]/magnitude_pred[indices], V_pred[indices]/magnitude_pred[indices], color='b', scale=50)
    axes[1].set_title('Predicted Wind Field')
    
    for ax in axes:
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.grid(True)
    
    # plt.show()


        
# Define the smaller area coordinates
lower_left = [-96.84, 33.14]#[-96.94, 33.06]
upper_right = [-96.81,33.17]#[-96.76, 33.22]

# Generate wind data for the smaller area
small_area_wind_data = generate_functional_wind_data_for_area(lower_left, upper_right, num_points=1000)

# Prepare the JSON structure
wind_data_json = {
    "area": {
        "lower_left": lower_left,
        "upper_right": upper_right
    },
    "wind_data": small_area_wind_data
}

# Define the path for the new JSON file
wind_data_path = 'dummy_wind_Frisco.json'

# Write the generated data to a new JSON file
with open(wind_data_path, 'w') as file:
    json.dump(wind_data_json, file, indent=4)

[poly_model_u, poly_model_v] =  plot_wind_field_with_contour(wind_data_path)

center_lon = (-96.84 + -96.81) / 2
center_lat = (33.14 + 33.17) / 2 
# plot_uav_trajectories_on_wind_contour(directory_path, center_lat, center_lon)

fitted_params_u = [1.16535633e-05, 5.54729748e-06]
fitted_params_v =  [-4.96795730e-05,  1.46530271e-05]
plot_comparison_and_error(wind_data_path, poly_model_u, poly_model_v, center_lat, center_lon)
directory_path = '/home/abenezertaye/Desktop/Research/Codes/Chapter-2/EnergyRequirementResults'

plot_uav_trajectories_on_wind_contour(directory_path, center_lat, center_lon)

