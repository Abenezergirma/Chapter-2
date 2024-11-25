import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from scipy.io import savemat
import pyproj
plt.rcParams['text.usetex'] = True


class WindDataProcessor:
    def __init__(self, file_path):
        self.file_path = file_path
        self.x = None
        self.y = None
        self.density = None
        self.x_momentum = None
        self.y_momentum = None
        self.wind_speed = None
        self.lat_reference = 42.256243  # Latitude reference
        self.lon_reference = -71.140111  # Longitude reference
        self.x_ref = 753600  # X coordinate of reference point
        self.y_ref = 2918650  # Y coordinate of reference point
        # Bounding box for area of interest
        self.lat_min = 42.230629
        self.lat_max = 42.259
        self.lon_min = -71.146753
        self.lon_max = -71.137655

        # Define the origin as the center of the bounding box
        self.lat_origin = (self.lat_min + self.lat_max) / 2
        self.lon_origin = (self.lon_min + self.lon_max) / 2

    def load_data(self):
        # Collect valid rows
        data = []
        with open(self.file_path, 'r') as file:
            for line in file:
                try:
                    line_split = line.strip().split()
                    if len(line_split) >= 5:
                        float(line_split[0])
                        data.append(line_split)
                except ValueError:
                    continue
        
        data = np.array(data, dtype=float)[:, [0, 1, 2, 3, 4]]

        self.x = data[:, 0]
        self.y = data[:, 1]
        self.density = data[:, 2]
        self.x_momentum = data[:, 3]
        self.y_momentum = data[:, 4]
        
    def compute_wind_speed(self):
        x_velocity = self.x_momentum / self.density
        y_velocity = self.y_momentum / self.density
        self.wind_speed = np.sqrt(x_velocity**2 + y_velocity**2)

    def convert_to_lat_lon(self):
        # Conversion factors
        delta_x = 753800 - 753600  # 200 units
        delta_lon = -71.139349 - (-71.140111)  # 0.000762 degrees
        lon_change_per_x = delta_lon / delta_x  # Degrees per unit x

        # Use actual data points for y and latitude
        y1 = 2918650
        y2 = 2918850  # Replace with actual value
        lat1 = 42.256243
        lat2 = 42.258000  # Replace with actual value

        delta_y = y2 - y1
        delta_lat = lat2 - lat1
        lat_change_per_y = delta_lat / delta_y  # Degrees per unit y

        # Convert x and y to lat and lon
        latitudes = self.lat_reference + (self.y - self.y_ref) * lat_change_per_y
        longitudes = self.lon_reference + (self.x - self.x_ref) * lon_change_per_x

        return latitudes, longitudes

    def filter_data_by_area(self, latitudes, longitudes):
        # Filter latitudes and longitudes within the specified area
        valid_indices = (
            (latitudes >= self.lat_min) & (latitudes <= self.lat_max) &
            (longitudes >= self.lon_min) & (longitudes <= self.lon_max)
        )
        return valid_indices

    def plot_wind_speed_heatmap_with_direction(self):
        if self.wind_speed is None:
            raise ValueError("Wind speed has not been computed yet. Please run compute_wind_speed() first.")
        
        latitudes, longitudes = self.convert_to_lat_lon()
        
        # Filter data based on rectangular area of interest
        valid_indices = self.filter_data_by_area(latitudes, longitudes)
        latitudes = latitudes[valid_indices]
        longitudes = longitudes[valid_indices]
        wind_speed = self.wind_speed[valid_indices]
        x_momentum = self.x_momentum[valid_indices]
        y_momentum = self.y_momentum[valid_indices]

        # Create grid for heatmap
        grid_lat, grid_lon = np.mgrid[
            latitudes.min():latitudes.max():100j,
            longitudes.min():longitudes.max():100j
        ]

        # Interpolate wind speed for heatmap
        grid_wind_speed = griddata(
            (latitudes, longitudes),
            wind_speed*0.75,
            (grid_lat, grid_lon),
            method='linear'
        )

        # Plot wind speed heatmap
        plt.figure(figsize=(4, 4))  # Set figure size to 4x4 inches
        plt.contourf(grid_lon, grid_lat, grid_wind_speed, cmap='YlGn')
        plt.colorbar(label='Wind Speed (m/s)')
        
        
                # Compute the x and y velocity components from the momentum
        x_velocity_filtered = (self.x_momentum[valid_indices] / self.density[valid_indices])
        y_velocity_filtered = (self.y_momentum[valid_indices] / self.density[valid_indices])
        
        # Quiver plot for wind direction
        # plt.quiver(longitudes, latitudes, x_momentum, y_momentum, color='blue')
        plt.quiver(longitudes, latitudes, x_velocity_filtered, y_velocity_filtered, color='blue')

        
        
        
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        # plt.title('Wind Speed and Direction in Latitude/Longitude')
        # plt.plot(self.lon_reference, self.lat_reference, marker='o', markersize=8, color='red', label='Reference Point')
        # plt.legend()

        # Save figure as PDF
        plt.savefig('wind_speed_direction_plot.png', format='png', bbox_inches='tight')
        plt.show()

    def convert_lat_lon_to_xy(self, latitudes, longitudes):
        # Convert lat/lon to x/y coordinates relative to the center of the bounding box
        transformer = pyproj.Transformer.from_proj(
            pyproj.Proj(init="epsg:4326"),  # WGS 84 lat/lon
            pyproj.Proj(init="epsg:3857")   # Mercator projection in meters
        )
        x_origin, y_origin = transformer.transform(self.lat_origin, self.lon_origin)
        x_coords, y_coords = transformer.transform(latitudes, longitudes)
        return x_coords - x_origin, y_coords - y_origin

    def export_to_mat(self, file_name):
        latitudes, longitudes = self.convert_to_lat_lon()
        valid_indices = self.filter_data_by_area(latitudes, longitudes)

        # Filtered data
        latitudes_filtered = latitudes[valid_indices]
        longitudes_filtered = longitudes[valid_indices]
        wind_speed_filtered = self.wind_speed[valid_indices]
        
        # Compute the x and y velocity components from the momentum
        x_velocity_filtered = (self.x_momentum[valid_indices] / self.density[valid_indices])
        y_velocity_filtered = (self.y_momentum[valid_indices] / self.density[valid_indices])

        # Convert filtered lat/lon to x/y coordinates
        x_filtered, y_filtered = self.convert_lat_lon_to_xy(latitudes_filtered, longitudes_filtered)

        # Create a dictionary for MATLAB file
        wind_data_dict = {
            'x': x_filtered,
            'y': y_filtered,
            'wind_speed': wind_speed_filtered,
            'x_velocity': x_velocity_filtered,
            'y_velocity': y_velocity_filtered
        }

        # Save the data to a .mat file
        savemat(file_name, wind_data_dict)
        print(f"Wind data saved to {file_name}")



# Example usage
file_path = 'case_a30.00000.dat' #'case_a105.00000.dat' #'case_m0.03100.dat'   # 

# Create an instance of the WindDataProcessor class
wind_processor = WindDataProcessor(file_path)

# Load the data, compute wind speed, and plot the heatmap and wind direction
wind_processor.load_data()
wind_processor.compute_wind_speed()
wind_processor.plot_wind_speed_heatmap_with_direction()

# Export the filtered wind data to a .mat file with x, y coordinates
wind_processor.export_to_mat("filtered_wind_data.mat")
