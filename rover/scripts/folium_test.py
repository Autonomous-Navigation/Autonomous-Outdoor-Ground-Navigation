#!/usr/bin/env python
# -*- coding: utf-8 -*-

import folium
from folium.plugins import MarkerCluster
import time
import random
from urllib.request import urlopen

# Function to generate random latitude and longitude for demo purposes
def generate_random_location():
    return (random.uniform(-90, 90), random.uniform(-180, 180))

# Create a folium map centered around a specific location
initial_location = (0, 0)  # You can set your own initial location
mymap = folium.Map(location=initial_location, zoom_start=2)

# Create a marker cluster for better performance with a large number of markers
marker_cluster = MarkerCluster().add_to(mymap)

# Update the map in real-time (for demo purposes)
for i in range(10):  # Update the map 10 times for demonstration
    location = generate_random_location()

    # Create a marker at the new location
    folium.Marker(location=location).add_to(marker_cluster)

    # Save the map as an HTML file
    mymap.save("realtime_map.html")

    # Pause for a short time (simulating real-time updates)
    time.sleep(2)

print("Real-time mapping demo completed. Check the generated HTML file.")

