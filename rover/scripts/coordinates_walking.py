import requests
#import folium
from geopy import distance


# Geocode the start and end points to get their addresses
def geocode(lat, lng):
	url = "https://nominatim.openstreetmap.org/reverse?format=json&lat={0}&lon={1}".format(lat, lng)
	response = requests.get(url).json()
	return response['display_name']
	


def fun_coordinates_walking(start_lat, start_lng, end_lat, end_lng):


	# Define the start and end points
	#start_lat = 33.6451434289607
	#start_lng = -117.84275116258073
	#end_lat = 33.64585943540695
	#end_lng = -117.84199088102766
	
	
	start_address = geocode(start_lat, start_lng)
	end_address = geocode(end_lat, end_lng)

	api_key = "5b3ce3597851110001cf624852531326e69d44d98933a46f80f14de7"
	
	# Build the request URL for the Mapbox API
	#url = f"https://api.mapbox.com/directions/v5/mapbox/walking/{start_lng},{start_lat};{end_lng},{end_lat}?geometries=geojson&access_token=pk.eyJ1Ijoicmlja3liZXZhbjUyIiwiYSI6ImNsaHJ6NWMxbzBkNDgzanFtcXh6MHRjbGgifQ.87Y6PMFUsopjO5MNi7Hicw"
	#url = "https://api.mapbox.com/directions/v5/mapbox/walking/{0},{1};{2},{3}?geometries=geojson&access_token=pk.eyJ1Ijoicmlja3liZXZhbjUyIiwiYSI6ImNsaHJ6NWMxbzBkNDgzanFtcXh6MHRjbGgifQ.87Y6PMFUsopjO5MNi7Hicw".format(start_lng, start_lat, end_lng, end_lat)
	url = "https://api.openrouteservice.org/v2/directions/cycling-regular?api_key={0}&start={1},{2}&end={3},{4}".format(api_key, start_lng, start_lat, end_lng, end_lat)

	# Send the request to the Mapbox API and parse the response
	response = requests.get(url)
	data = response.json()
	
	# Extract the latitude and longitude of each point in the path
	path_points = data['features'][0]['geometry']['coordinates']
	
	# Add a new point every 5 meters along each segment of the path
	new_path_points = []
	for i in range(len(path_points) - 1):
			p1 = path_points[i]
			p2 = path_points[i + 1]
			dist = distance.distance((p1[1], p1[0]), (p2[1], p2[0])).meters

			if dist <= 5:
				new_path_points.append([p1[1], p1[0]])  # switch longitude and latitude
			else:
				n_points = int(dist / 40)  # Number of points for every 5 meters
				if n_points == 0:
					break
				interval_lat = (p2[1] - p1[1]) / n_points
				interval_lng = (p2[0] - p1[0]) / n_points

				for j in range(n_points + 1):
						lat = p1[1] + (interval_lat * j)
						lng = p1[0] + (interval_lng * j)
						new_path_points.append([lat, lng])  # switch longitude and latitude

	
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
	#print(len(new_path_points))
	return(new_path_points)
	# Display the map
	#m 
