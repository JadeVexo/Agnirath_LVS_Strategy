import asyncio
import random
from math import radians, sin, cos, sqrt
import websockets

# Specify the center coordinates
center_latitude = 12.997036661158213
center_longitude = 80.23221913617647

# Specify the radius in meters
radius = 500

async def send_coordinates(websocket, path):
    while True:
        # Generate random distances in meters
        dx = random.uniform(-radius, radius)
        dy = random.uniform(-radius, radius)

        # Convert meters to degrees using a rough approximation
        latitude_change = dy / 111111
        longitude_change = dx / (111111 * cos(radians(center_latitude)))

        # Calculate the new coordinates
        new_latitude = center_latitude + latitude_change
        new_longitude = center_longitude + longitude_change

        # Create a string message with the new coordinates
        message = f"{new_longitude},{new_latitude}"
        print(message)

        # Send the message to the client
        await websocket.send(message)

        await asyncio.sleep(0.5)

start_server = websockets.serve(send_coordinates, 'localhost', 8888)

asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()
