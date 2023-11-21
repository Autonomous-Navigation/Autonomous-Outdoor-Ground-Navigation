import requests
#import folium
from geopy import distance

# Define the start and end points
start_lat = 33.645142
start_lng = -117.842741
end_lat = 33.644772
end_lng = -117.842121

# OpenRouteService API Key
api_key = "5b3ce3597851110001cf624852531326e69d44d98933a46f80f14de7"  # Replace this with your OpenRouteService API Key

# Geocode the start and end points to get their addresses
def geocode(lat, lng):
    url = "https://nominatim.openstreetmap.org/reverse?format=json&lat={0}&lon={1}".format(lat, lng)
    response = requests.get(url).json()
    return response['display_name']

#start_address = geocode(start_lat, start_lng)
#end_address = geocode(end_lat, end_lng)

# Build the request URL for the OpenRouteService API
url = "https://api.openrouteservice.org/v2/directions/cycling-regular?api_key={0}&start={1},{2}&end={3},{4}".format(
    api_key, start_lng, start_lat, end_lng, end_lat)

# Send the request to the OpenRouteService API and parse the response
response = requests.get(url)
data = response.json()

print(data)

# Extract the latitude and longitude of each point in the path
path_points = data['features'][0]['geometry']['coordinates']

# Add a new point every 5 meters along each segment of the path
new_path_points = []
for i in range(len(path_points) - 1):
    p1 = path_points[i]
    p2 = path_points[i + 1]
    dist = distance.distance((p1[1], p1[0]), (p2[1], p2[0])).meters

    if dist <= 5:
        new_path_points.append(p1)
    else:
        n_points = int(dist / 5)  # Number of points for every 5 meters
        interval_lat = (p2[1] - p1[1]) / n_points
        interval_lng = (p2[0] - p1[0]) / n_points

        for j in range(n_points + 1):
            lat = p1[1] + (interval_lat * j)
            lng = p1[0] + (interval_lng * j)
            new_path_points.append([lng, lat])

# Create a Folium map centered on the start point
map_center = [start_lat, start_lng]
#m = folium.Map(location=map_center, zoom_start=13)

# Add markers for the start and end points
#folium.Marker(location=[start_lat, start_lng], popup=start_address).add_to(m)
#folium.Marker(location=[end_lat, end_lng], popup=end_address).add_to(m)

# Add a polyline for the bicycle route between the start and end points
path_coords = [(point[1], point[0]) for point in new_path_points]
#folium.PolyLine(locations=path_coords, color='green').add_to(m)

print(new_path_points)
print(len(new_path_points))# Define a mapping of directions to numbers
direction_mapping = {
    'left': 1,
    'straight': 2,
    'right': 3,
    'head': 2,  # consider 'head' as 'straight'
    'roundabout': 4,
}

# Extract the turn-by-turn instructions, distances, and their coordinates
steps = data['features'][0]['properties']['segments'][0]['steps']

# Initialize an empty list to store instructions
instructions_list = []

for step in steps:
    location = step['way_points'][0]
    coords = path_points[location]
    instruction = step['instruction'].lower()  # Convert instruction to lowercase for matching
    distance_value = step['distance']

    # Determine the numeric direction value
    direction_number = 0  # Default to 0 if no match is found
    for direction, number in direction_mapping.items():
        if direction in instruction:
            direction_number = number
            break

    # Append the direction number and distance as a sublist
    instructions_list.append([direction_number, distance_value])

print(instructions_list)

# Display the map
