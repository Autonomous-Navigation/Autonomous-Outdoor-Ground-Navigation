import requests
#import folium
import numpy as np
		

def fun(start_lat, start_lng, end_lat, end_lng):

# Define the start and end points
	#start_lat =33.64514403550909 
	#start_lng =  -117.84273063954905
	#end_lat = 33.64585607700809
	#end_lng = -117.84198898453141

# Geocode the start and end points to get their addresses
	start_url = "https://nominatim.openstreetmap.org/reverse?format=json&lat={0}&lon={1}".format(start_lat, start_lng)
	end_url = "https://nominatim.openstreetmap.org/reverse?format=json&lat={0}&lon={1}".format(end_lat, end_lng)

	start_response = requests.get(start_url).json()
	end_response = requests.get(end_url).json()
	
	start_address = start_response['display_name']
	end_address = end_response['display_name']
	
	# Build the request URL for the OpenStreetMap API
	url = "https://router.project-osrm.org/route/v1/driving/{0},{1};{2},{3}?overview=full&geometries=geojson".format(
	    start_lng, start_lat, end_lng, end_lat)
	
	# Send the request to the OpenStreetMap API and parse the response
	response = requests.get(url)
	data = response.json()
	
	# Extract the latitude and longitude of each point in the path
	path_points = data['routes'][0]['geometry']['coordinates']
	
	# Add a new point every meter along each segment of the path
	new_path_points = []
	for i in range(len(path_points)-1):
	    p1 = path_points[i]
	    p2 = path_points[i+1]
	    dist = np.sqrt((p2[0]-p1[0])**2 + (p2[1]-p1[1])**2)
	    n_points = int(np.ceil(dist))
	    for j in range(n_points):
	        ratio = j / n_points
	        lat = p1[1] * (1-ratio) + p2[1] * ratio
	        lng = p1[0] * (1-ratio) + p2[0] * ratio
	        new_path_points.append([lat, lng])
	
	# Create a Folium map centered on the start point
	#map_center = [start_lat, start_lng]
	#m = folium.Map(location=map_center, zoom_start=13)
	
	# Add markers for the start and end points
	#folium.Marker(location=[start_lat, start_lng], popup=start_address).add_to(m)
	#folium.Marker(location=[end_lat, end_lng], popup=end_address).add_to(m)
	
	# Add a polyline for the path between the start and end points
	#path_coords = [(point[1], point[0]) for point in new_path_points]
	#folium.PolyLine(locations=path_coords, color='blue').add_to(m)
	#print(new_path_points)
	return(new_path_points)
	# Display the map
	#	m



