import folium

# Initialize the map
map = folium.Map(location=[32.863498, -96.803123], zoom_start=13)

# folium.TileLayer('CartoDB Positron').add_to(map)
# Warehouse and destination positions
Depots = [[32.872522864864862, -96.792586105205572], [32.852581243243243, -96.795005230041056], [32.852164936936937, -96.814858785391237]]
Destinations = [[32.864085567567571, -96.806916269255453], [32.862029441441443, -96.794711460015591], [32.863482234234233, -96.815022884748288], 
                [32.852869441441442, -96.804653628708820], [32.863255657657660, -96.799023948217126], [32.861800072072072, -96.814951882281392]]

# Add markers and labels for warehouses
for location in Depots:
    folium.Marker(location, icon=folium.Icon(color='blue', icon='building', prefix='fa')).add_to(map)
    label = folium.map.Marker(
        location,
        icon=folium.DivIcon(
            html=f'<div style="font-size: 14px; font-weight: bold; text-align: center;">Depots</div>'
        )
    )
    label.add_to(map)

# Add markers and labels for destinations
for location in Destinations:
    folium.Marker(location, icon=folium.Icon(color='green', icon='flag-checkered', prefix='fa')).add_to(map)
    label = folium.map.Marker(
        location,
        icon=folium.DivIcon(
            html=f'<div style="font-size: 14px; font-weight: bold; text-align: center;">Destination</div>'
        )
    )
    label.add_to(map)

# Define corners of the rectangle for the border
corner_lat = [32.875395, 32.875395, 32.851601, 32.851601]
corner_lon = [-96.788959, -96.817287, -96.817287, -96.788959]
rectangle_corners = [(corner_lat[i], corner_lon[i]) for i in range(len(corner_lat))]

# Add a polygon for the rectangular border without fill
folium.Polygon(rectangle_corners, color='red', fill=False).add_to(map)

# Save the map to an HTML file
map.save('map.html')


