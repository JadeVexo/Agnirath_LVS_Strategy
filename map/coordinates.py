import csv
from datetime import datetime, timedelta
import time

## asdasdasdasda
# Define the list of coordinates
coordinates = [
    [80.23221913617647, 12.997036661158213],
    [80.23521913617647, 12.998036661158213],
    [80.23121913617647, 12.993036661158213],
    [80.23621913617647, 12.992036661158213],
    [80.23721913617647, 12.991036661158213],
    [80.23321913617647, 12.998036661158213],
    [80.23521913617647, 12.993036661158213],
    [80.23521913617647, 12.999036661158213],
    [80.23721913617647, 12.996036661158213],
    [80.23121913617647, 12.999036661158213],
    [80.23821913617647, 12.991036661158213],
    [80.23221913617647, 12.992036661158213],
    [80.23921913617647, 12.998036661158213],
    [80.23221913617647, 12.999036661158213],
    [80.23721913617647, 12.994036661158213],
    [80.23321913617647, 12.990036661158213],
    [80.23921913617647, 12.993036661158213],
    [80.23021913617647, 12.992036661158213],
    [80.23221913617647, 12.998036661158213],
    [80.23121913617647, 12.998036661158213]
]

# Specify the output file name
output_file = 'coordinates.csv'

# Write the coordinates to the CSV file with a 0.5-second interval
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for coordinate in coordinates:
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        row = [current_time] + coordinate
        writer.writerow(row)
        time.sleep(0.5)

print("Coordinates have been written to", output_file)
