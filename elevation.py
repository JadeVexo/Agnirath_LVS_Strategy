import requests

def get_coordinates(api_key, origin, destination, waypoints=None):
    base_url = 'https://maps.googleapis.com/maps/api/directions/json?'

    # Create the waypoints parameter if specified
    waypoints_param = ''
    if waypoints:
        waypoints_param = '&'.join(f'waypoints={wp}' for wp in waypoints)

    # Construct the request URL
    request_url = f'{base_url}origin={origin}&destination={destination}&{waypoints_param}&key={api_key}'

    # Send the request to the API
    response = requests.get(request_url)
    data = response.json()

    # Extract the coordinates from the response
    coordinates = []
    if data['status'] == 'OK':
        routes = data['routes']
        for route in routes:
            legs = route['legs']
            for leg in legs:
                steps = leg['steps']
                for step in steps:
                    start_location = step['start_location']
                    end_location = step['end_location']
                    coordinates.append((start_location['lat'], start_location['lng']))
                    coordinates.append((end_location['lat'], end_location['lng']))

    return coordinates

# Specify your API key, origin, destination, and waypoints
API_KEY = 'AIzaSyDKKVK8NtYOa_DDPafADUeISiJR3eIm4ig'
origin = 'Chennai, India'
destination = 'Pondicherry, India'
waypoints = ['Villupuram, India', 'Cuddalore, India']

# Call the function to get the coordinates
route_coordinates = get_coordinates(API_KEY, origin, destination, waypoints)

# Print the coordinates
for coordinate in route_coordinates:
    print(coordinate)
