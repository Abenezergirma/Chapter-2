import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
# Enable LaTeX in Matplotlib
plt.rcParams['text.usetex'] = True

# Assuming wind data points are stored similarly and need to be interpolated or generated for a new area
def generate_wind_data_for_area(lower_left, upper_right, num_points=10):
    """
    Generates placeholder wind data for an area defined by diagonal coordinates.
    
    Args:
    - lower_left: Tuple of (longitude, latitude) for the lower left corner.
    - upper_right: Tuple of (longitude, latitude) for the upper right corner.
    - num_points: Number of data points to generate along each axis.
    
    Returns:
    - List of wind data points with coordinates and placeholder wind properties.
    """
    lons = np.linspace(lower_left[0], upper_right[0], num_points)
    lats = np.linspace(lower_left[1], upper_right[1], num_points)
    
    wind_data = []
    for lon in lons:
        for lat in lats:
            wind_data.append({
                "coordinates": [lon, lat],
                "wind_speed": np.random.uniform(8, 13),  # Placeholder for wind speed
                "wind_direction": np.random.uniform(200, 220)
                # "wind_direction": np.random.choice(['N', 'E', 'S', 'W'])  # Placeholder for wind direction
            })
    
    return wind_data

def plot_wind_field_with_contour(file_path, vector_scale=60, subsample_rate=3):
    try:
        with open(file_path, 'r') as file:
            data = json.load(file)
        
        wind_data = data.get('wind_data', [])
        if wind_data:
            # Extract longitude, latitude, wind speed, and wind direction from the data
            longitudes = np.array([point['coordinates'][0] for point in wind_data])
            latitudes = np.array([point['coordinates'][1] for point in wind_data])
            wind_speeds = np.array([point['wind_speed'] for point in wind_data])
            wind_directions = np.array([point['wind_direction'] for point in wind_data])

            # Convert wind direction and speed into U and V components
            U = wind_speeds * np.cos(np.deg2rad(wind_directions))
            V = wind_speeds * np.sin(np.deg2rad(wind_directions))

            # Normalize the wind vectors for uniform arrow size
            magnitude = np.sqrt(U**2 + V**2)
            U_norm = U / magnitude
            V_norm = V / magnitude

            # Create a grid for the contour plot
            grid_x, grid_y = np.mgrid[min(longitudes):max(longitudes):100j, min(latitudes):max(latitudes):100j]
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
        
# Define the smaller area coordinates
lower_left = [-96.94, 33.06]
upper_right = [-96.76, 33.22]

# Generate wind data for the smaller area
small_area_wind_data = generate_wind_data_for_area(lower_left, upper_right, num_points=100)

# Prepare the JSON structure
wind_data_json = {
    "area": {
        "lower_left": lower_left,
        "upper_right": upper_right
    },
    "wind_data": small_area_wind_data
}

# Define the path for the new JSON file
wind_data_path = 'wind_data_DFW_at_0_5km_new.json'

# Write the generated data to a new JSON file
with open(wind_data_path, 'w') as file:
    json.dump(wind_data_json, file, indent=4)

plot_wind_field_with_contour(wind_data_path)
