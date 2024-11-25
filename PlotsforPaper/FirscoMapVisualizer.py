import folium
from folium import IFrame, Marker
from folium.plugins import Draw
from lxml import etree
import geopandas as gpd
from shapely.geometry import Point, LineString, box
import os
import streamlit as st
from streamlit_folium import st_folium
import pandas as pd
import scipy    


class FriscoMapVisualizer:
    def __init__(self):
        self.radius = 100  # The radius for the circles around each point in meters
        self.walmart_locations = [
            ("Origin", "Origin 1", 42.253617, -71.143513, self.radius),
            ("Destination 1", "Destination 1", 42.255935, -71.138921, self.radius),
            ("Destination 2", "Destination 2", 42.249582, -71.144135, self.radius)
            # ("Walmart 4", "16066 State Hwy 121, Frisco, TX 75035", 33.127910, -96.736370, self.radius),
        ]

    def create_map_with_draw_and_locations(self):
        m = folium.Map(location=[42.253617, -71.143513], zoom_start=13)
        draw = Draw(export=True)
        draw.add_to(m)

        for name, address, lat, lon, radius in self.walmart_locations:
            folium.Marker(
                location=[lat, lon],
                popup=f"{name}: {address}",
                icon=folium.Icon(icon="info-sign")
            ).add_to(m)

            folium.Circle(
                location=[lat, lon],
                radius=radius,
                popup=f"Radius: {radius}m around {name}",
                color='blue',
                fill=False,
                fill_color='blue'
            ).add_to(m)

        return m

    def plot_kml_on_map_old(self, m, kml_file_path):
        with open(kml_file_path, 'r', encoding='utf-8') as file:
            kml_content = file.read()
        root = etree.fromstring(kml_content.encode('utf-8'))

        ns = '{http://www.opengis.net/kml/2.2}'

        for element in root.findall('.//{}LineString'.format(ns)) + root.findall('.//{}Polygon'.format(ns)):
            coords = element.find('.//{}coordinates'.format(ns))
            if coords is not None:
                coord_pairs = coords.text.strip().split(' ')
                parsed_coords = [(float(lat), float(lon)) for lon, lat, *_ in (pair.split(',') for pair in coord_pairs)]

                if element.tag == f"{ns}LineString":
                    folium.PolyLine(locations=parsed_coords).add_to(m)
                elif element.tag == f"{ns}Polygon":
                    folium.Polygon(locations=[parsed_coords]).add_to(m)

        return m

    def convert_to_geojson(self):
        points_gdf = gpd.GeoDataFrame(
            self.walmart_locations,
            columns=['Name', 'Address', 'Latitude', 'Longitude', 'Radius'],
            geometry=[Point(lon, lat) for _, _, lat, lon, _ in self.walmart_locations],
            crs="EPSG:4326"
        )

        points_gdf = points_gdf.to_crs(epsg=3857)
        circles_gdf = points_gdf.copy()
        circles_gdf['geometry'] = circles_gdf.apply(lambda x: x['geometry'].buffer(x['Radius']).boundary, axis=1)
        circles_gdf = circles_gdf.to_crs(epsg=4326)

        points_filepath = 'walmart_locations.geojson'
        circles_filepath = 'walmart_locations_circles.geojson'
        points_gdf.to_file(points_filepath, driver='GeoJSON')
        circles_gdf.to_file(circles_filepath, driver='GeoJSON')

        return points_filepath, circles_filepath
    
    def add_rectangular_markers_to_lines(self, geojson_input_path, geojson_output_path):
        # Load the GeoJSON file
        gdf = gpd.read_file(geojson_input_path)

        # Filter LineString geometries (assuming these are the user-drawn lines)
        lines_gdf = gdf[gdf['geometry'].type == 'LineString']

        # Create a list to store rectangular markers
        marker_geometries = []

        # For each LineString, place rectangular markers on its coordinates
        for line in lines_gdf.geometry:
            for coord in list(line.coords):
                # Creating a small rectangular marker around each coordinate
                # Note: Adjust the buffer values as needed for the size of the rectangles
                marker = box(coord[0] - 0.0001, coord[1] - 0.0001, coord[0] + 0.0001, coord[1] + 0.0001)
                marker_geometries.append(marker)

        # Create a GeoDataFrame for rectangular markers
        markers_gdf = gpd.GeoDataFrame(geometry=marker_geometries, crs="EPSG:4326")

        # Combine the original GeoDataFrame with the new markers GeoDataFrame
        combined_gdf = pd.concat([gdf, markers_gdf], ignore_index=True)
        combined_gdf = gpd.GeoDataFrame(combined_gdf, crs="EPSG:4326")

        # Export to a new GeoJSON file
        combined_gdf.to_file(geojson_output_path, driver='GeoJSON')

        return geojson_output_path
    
    def waypoints_to_geojson(self, waypoints, filename='uav_trajectory.geojson'):
        """
        Convert a list of 3D waypoints into a GeoJSON file.
        Parameters:
        - waypoints: A list of (x, y, z) tuples representing the UAV's trajectory.
        - filename: The name of the output GeoJSON file.
        """
        # Convert the waypoints to a LineString, ignoring the z-coordinate for simplicity.
        # If you need to include the z-coordinate in the visualization, you may need a 3D-aware tool or software.
        line = LineString([(point[0], point[1]) for point in waypoints])

        # Create a GeoDataFrame
        gdf = gpd.GeoDataFrame([1], geometry=[line], crs="EPSG:4326")
        gdf.columns = ['id', 'geometry']

        # Export to GeoJSON
        gdf.to_file(filename, driver='GeoJSON')
        print(f"GeoJSON file saved as {filename}")

if __name__ == "__main__":
    visualizer = FriscoMapVisualizer()
    map_with_draw = visualizer.create_map_with_draw_and_locations()
    # kml_file_path = 'FriscoMunBnd_ODH.kml'
    # map_with_draw = visualizer.plot_kml_on_map_old(map_with_draw, kml_file_path)
    map_with_draw.save('boston_map_with_draw.html')
    # geojson_paths = visualizer.convert_to_geojson()
    # geojson_output_path = 'LineMarkers.geojson'
    # geojson_input_path = 'data (10).geojson'
    # visualizer.add_rectangular_markers_to_lines(geojson_input_path, geojson_output_path)
    # wayPointsnew = scipy.io.loadmat('uav_traj.mat')
    # print(wayPointsnew['uav_traj'])
    # # Convert and save the UAV trajectory as GeoJSON
    # visualizer.waypoints_to_geojson(wayPointsnew['uav_traj'])
