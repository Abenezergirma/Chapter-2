import os
import xml.etree.ElementTree as ET
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
# from mpl_toolkits.basemap import Basemap
# import folium
import matplotlib.animation as animation
from matplotlib.patches import PathPatch
from matplotlib.path import Path
import numpy as np
import geopandas as gpd
import pandas as pd
from shapely.geometry import Point, LineString
import contextily as ctx
import cartopy.crs as ccrs
import cartopy.feature as cfeature


plt.rcParams['text.usetex'] = True



class UAVTrajectoryAnimator:
    def __init__(self, directory):
        """Initialize the animator with the directory containing KML files."""
        self.directory = directory
        self.trajectories = []
        self.area = None
        self.destinations = []
        
    def parse_kml(self, file):
        """Parse KML file and extract coordinates."""
        tree = ET.parse(file)
        root = tree.getroot()

        ns = {'kml': 'http://www.opengis.net/kml/2.2'}
        coord_elements = root.findall('.//kml:Placemark/kml:Point/kml:coordinates', ns) or \
                         root.findall('.//kml:Placemark/kml:LineString/kml:coordinates', ns)

        all_coords = []
        for element in coord_elements:
            coords = element.text.strip().split()
            all_coords.extend([(float(lon), float(lat)) for lon, lat, _ in (point.split(',') for point in coords)])

        return all_coords

    def load_data(self):
        """Load trajectory data from KML files in the specified directory."""
        for filename in os.listdir(self.directory):
            filepath = os.path.join(self.directory, filename)
            if filename.startswith('uav'):
                self.trajectories.append(self.parse_kml(filepath))
            # Add similar logic for area and destinations if needed

    def sample_trajectories(self, step=100):
        """Sample the trajectories to reduce the number of points."""
        self.trajectories = [trajectory[::step] for trajectory in self.trajectories]

    def get_coordinate_range(self):
        """Get the coordinate range (min and max) for plotting."""
        all_lons = [lon for trajectory in self.trajectories for lon, lat in trajectory]
        all_lats = [lat for trajectory in self.trajectories for lon, lat in trajectory]
        return min(all_lons), max(all_lons), min(all_lats), max(all_lats)
    
    def plot_trajectories(self):
        """Plot the trajectories on a static map."""
        fig, ax = plt.subplots()
        for i, trajectory in enumerate(self.trajectories):
            lons, lats = zip(*trajectory)
            ax.plot(lons, lats, label=f'UAV {i+1}')

        # Similar plotting logic for area and destinations
        ax.set_xlabel('Longitude')
        ax.set_ylabel('Latitude')
        ax.set_title('UAV Trajectories')
        ax.legend()
        plt.show()
        
    def plot_trajectories_on_map(self):
        """Plot the trajectories on an actual map."""
        fig, ax = plt.subplots()

        # Create a Basemap instance
        min_lon, max_lon, min_lat, max_lat = self.get_coordinate_range()
        m = Basemap(projection='merc', llcrnrlat=min_lat, urcrnrlat=max_lat, 
                    llcrnrlon=min_lon, urcrnrlon=max_lon, resolution='i')
        
        m.drawcoastlines()
        m.drawcountries()
        m.fillcontinents(color='lightgray', lake_color='aqua')
        m.drawmapboundary(fill_color='aqua')

        # Plot each UAV trajectory
        for i, trajectory in enumerate(self.trajectories):
            if trajectory:    
                lons, lats = zip(*trajectory)
                x, y = m(lons, lats)
                m.plot(x, y, marker=None, label=f'UAV {i+1}')

        # Similar plotting logic for area and destinations
        plt.title('UAV Trajectories on Map')
        plt.legend()
        plt.show()
        
    def animate_with_map(self, map_image_path, extent):

    #"""Animate the UAV trajectories on top of a map image."""
    # Load the map image
        map_image = plt.imread(map_image_path)

        # Create the plot
        fig, ax = plt.subplots()
        ax.imshow(map_image, extent=extent)  # Set the extent to the geographical bounds of the map
        ax.set_xlim(extent[0], extent[1])  # Longitude bounds
        ax.set_ylim(extent[2], extent[3])  # Latitude bounds

        # Initialize lines for UAV trajectories
        lines = [ax.plot([], [], 'o-', label=f'UAV {i+1}')[0] for i in range(len(self.trajectories))]

        def init():
            for line in lines:
                line.set_data([], [])
            return lines

        def animate(frame):
            for line, trajectory in zip(lines, self.trajectories):
                if frame < len(trajectory):
                    lons, lats = zip(*trajectory[:frame+1])
                    line.set_data(lons, lats)
            return lines

        ani = FuncAnimation(fig, animate, init_func=init, frames=max(len(t) for t in self.trajectories), blit=True)
        plt.show()
        
    def plot_trajectories_with_geopandas(self):
        """Plot the trajectories using Geopandas."""
        fig, ax = plt.subplots()

        # Convert trajectories to a GeoDataFrame
        gdf_list = []
        for i, trajectory in enumerate(self.trajectories):
            # Create a DataFrame for each trajectory
            df = pd.DataFrame(trajectory, columns=['lon', 'lat'])
            df['UAV'] = f'UAV {i+1}'
            
            # Convert DataFrame to GeoDataFrame
            gdf = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.lon, df.lat))
            gdf_list.append(gdf)

        # Combine all GeoDataFrames
        all_trajectories_gdf = pd.concat(gdf_list)
            # Set the CRS for GeoDataFrame to WGS84 (EPSG:4326)
        all_trajectories_gdf.set_crs(epsg=4326, inplace=True)

        # Reproject to Web Mercator for contextily basemap
        all_trajectories_gdf = all_trajectories_gdf.to_crs(epsg=3857)

        # Plot using Geopandas
        # all_trajectories_gdf.plot(ax=ax, column='UAV', legend=True, marker='o')
            # Set attractive colors for the trajectories
        colors = ['red', 'green', 'blue', 'purple', 'orange', 'cyan', 'magenta']

        # Plot each trajectory with a thinner line and different color
        for i, (label, df) in enumerate(all_trajectories_gdf.groupby('UAV')):
            df.plot(ax=ax, color='blue', legend=True, linewidth=1.0, marker='o', markersize=1.5)

        
        # Add Basemap using a different tile provider
        # ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
        print(ctx.providers.HEREv3.keys())
        
        # ctx.add_basemap(ax, source=ctx.providers.HEREv3.normalDay)
        # Example of how to update the tile provider with your token
        ctx.providers.HEREv3.normalDayCustom.build_url = lambda x, y, z: f"https://1.base.maps.ls.hereapi.com/maptile/2.1/maptile/newest/normal.day/{z}/{x}/{y}/256/png?apiKey=JOwwg7YVdrxTLyd49k85cQ"


        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('UAV Trajectories')
        plt.savefig('UAV_traj.png', dpi=300, bbox_inches='tight')
        plt.show()

    def animate_trajectories_with_quadrotor(self):
        """Animate UAV trajectories with quadrotor icons and save the recording."""
        # fig, ax = plt.subplots()
        fig, ax = plt.subplots(subplot_kw={'projection': ccrs.PlateCarree()})


        # Convert trajectories to a GeoDataFrame
        gdf_list = []
        for i, trajectory in enumerate(self.trajectories):
            df = pd.DataFrame(trajectory, columns=['lon', 'lat'])
            df['UAV'] = f'UAV {i+1}'
            gdf = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.lon, df.lat))
            gdf_list.append(gdf)
        
        # Combine all GeoDataFrames
        all_trajectories_gdf = pd.concat(gdf_list)
        all_trajectories_gdf.set_crs(epsg=4326, inplace=True)
        all_trajectories_gdf = all_trajectories_gdf.to_crs(epsg=3857)

        # Add Basemap
        # ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
        # Set a default zoom level
        default_zoom = 15  # Adjust this based on your data's extent

        # Add Basemap with a fixed zoom level
        ctx.add_basemap(ax, source=ctx.providers.OpenStreetMap.Mapnik)
        
        # Add geographical features
        ax.add_feature(cfeature.COASTLINE)
        ax.add_feature(cfeature.BORDERS, linestyle=':')


        # Function to create a quadrotor icon
        def create_quadrotor_icon():
            verts = [(0.0, -0.1), (0.0, 0.1), (0.1, 0.2), (0.1, -0.2), (0.0, -0.1)]
            codes = [Path.MOVETO, Path.LINETO, Path.LINETO, Path.LINETO, Path.CLOSEPOLY]
            path = Path(verts, codes)
            patch = PathPatch(path, facecolor='black', lw=2)
            return patch

        # Initialize quadrotor icons for each UAV
        quadrotors = [create_quadrotor_icon() for _ in self.trajectories]
        for quadrotor in quadrotors:
            ax.add_patch(quadrotor)
        
        def move_quadrotor(patch, new_position):
        # Get the current vertices of the patch
            vertices = patch.get_path().vertices
            # Calculate the offset
            offset = new_position - vertices.mean(axis=0)
            # Update the vertices
            vertices += offset

        
        # Set the geographic extent of the plot
        # ax.set_extent([min_lon, max_lon, min_lat, max_lat], crs=ccrs.PlateCarree())
        def update(frame):
            for i, trajectory in enumerate(self.trajectories):
                if frame < len(trajectory):
                    lon, lat = trajectory[frame]
                    # Convert the lon/lat to the axes coordinate system
                    x, y = ax.projection.transform_point(lon, lat, src_crs=ccrs.Geodetic())
                    # Move the quadrotor icon
                    move_quadrotor(quadrotors[i], np.array([x, y]))
            return quadrotors

        fig.canvas.draw()
        ani = FuncAnimation(fig, update, frames=max(len(t) for t in self.trajectories), blit=False)

        # Save the animation
        ani.save('uav_trajectories_animation.mp4', writer='ffmpeg', fps=30)
        plt.show()    
        
            
    def generate_folium_map_with_trajectories(self, depots, destinations, rectangle_corners, map_center, zoom_start=13):
        """Generate a Folium map with depots, destinations, and UAV trajectories."""
        # Initialize the map
        folium_map = folium.Map(location=map_center, zoom_start=zoom_start)
        
        # Define a list of colors for the trajectories
        colors = [ 'blue', 'green', 'purple', 'orange', 'darkred', 
                  'darkblue', 'darkgreen', 'cadetblue', 'lightred', 'beige', 
                  'darkpurple', 'white', 'red','pink', 'lightblue', 'lightgreen', 
                  'gray', 'black', 'lightgray']

        # Add markers for depots
        for location in depots:
            folium.Marker(location, icon=folium.Icon(color='blue', icon='building', prefix='fa')).add_to(folium_map)
            label = folium.map.Marker(
                location,
                icon=folium.DivIcon(
                html=f'<div style="font-size: 14px; font-weight: bold; text-align: center;">Depot</div>'
                )
            )
            label.add_to(folium_map)
            

        # Add markers for destinations
        for location in destinations:
            folium.Marker(location, icon=folium.Icon(color='green', icon='flag-checkered', prefix='fa')).add_to(folium_map)
            label = folium.map.Marker(
                location,
                icon=folium.DivIcon(
                html=f'<div style="font-size: 14px; font-weight: bold; text-align: center;">Destination</div>'
                )
            )
            label.add_to(folium_map)

        # Add a polygon for the rectangular area
        folium.Polygon(rectangle_corners, color='red', fill=False).add_to(folium_map)
    
        # Add UAV trajectories
        for i, trajectory in enumerate(self.trajectories):
            # Ensure the trajectory is in the format [(lat, lon), (lat, lon), ...]
            color = colors[i % len(colors)]  # Cycle through colors
            trajectory = [(lat, lon) for lon, lat in trajectory]
            folium.PolyLine(trajectory, color=color, weight=3.5, opacity=1).add_to(folium_map)


        # Save or show the map
        folium_map.save('map_with_trajectories.html')
        return folium_map
    
    def animate(self):
        """Animate the UAV trajectories."""
        fig, ax = plt.subplots()
        min_lon, max_lon, min_lat, max_lat = self.get_coordinate_range()
        ax.set_xlim(min_lon, max_lon)
        ax.set_ylim(min_lat, max_lat)
        ax.set_aspect('equal', adjustable='box')

        lines = [ax.plot([], [], 'o-', label=f'UAV {i+1}')[0] for i in range(len(self.trajectories))]

        def init():
            for line in lines:
                line.set_data([], [])
            return lines

        def update(frame):
            for line, trajectory in zip(lines, self.trajectories):
                line.set_data(*zip(*trajectory[:frame+1]) if frame < len(trajectory) else zip(*trajectory[-1:]))
            return lines
        
        ani = FuncAnimation(fig, update, frames=max(len(t) for t in self.trajectories), 
                            init_func=init, blit=True, interval=10)
        plt.legend()
        plt.show()

# Usage example
current_directory = os.path.dirname(os.path.abspath(__file__))
trajectory_directory = os.path.join(current_directory, "..", "TrajectoryPlanningResults/AircraftTrajectories")
animator = UAVTrajectoryAnimator(trajectory_directory)
animator.load_data()
animator.sample_trajectories(step=100)
# animator.animate_trajectories_with_quadrotor()
animator.plot_trajectories_with_geopandas()

map_image_path = 'background_map.png'
map_extent = [-96.817287, -96.788959, 32.851601, 32.875395]  # Replace with actual bounds
# animator.animate_with_map(map_image_path, map_extent)
# Define the depots, destinations, and the corners of the rectangle
Depots = [[32.872522864864862, -96.792586105205572], [32.852581243243243, -96.795005230041056], [32.852164936936937, -96.814858785391237]]
Destinations = [[32.864085567567571, -96.806916269255453], [32.862029441441443, -96.794711460015591], [32.863482234234233, -96.815022884748288], 
                [32.852869441441442, -96.804653628708820], [32.863255657657660, -96.799023948217126], [32.861800072072072, -96.814951882281392]]
rectangle_corners = [(32.875395, -96.788959), (32.875395, -96.817287), (32.851601, -96.817287), (32.851601, -96.788959)]

# Create and save the Folium map
# folium_map = animator.generate_folium_map_with_trajectories(Depots, Destinations, rectangle_corners, map_center=[32.863498, -96.803123], zoom_start=13)

# animator.plot_trajectories_on_map()
# animator.animate()

# corner_lat = [32.875395, 32.875395, 32.851601, 32.851601]
# corner_lon = [-96.788959, -96.817287, -96.817287, -96.788959]
