import random
import time
final_data = [0]*9  # Initialize the final_data list

def generate():
    data_type = random.randint(0, 2)  # Randomly select the data type: 0, 1, or 2
    values = [random.randint(0, 10) for i in range(3)]  # Generate random x, y, z components
    
    return [data_type] + values

while True:
    # Get the updated values from the generate function
    new_data = generate()
    
    # Update final_data list with new values based on data_type
    if new_data[0] == 0:
        final_data[:3] = new_data[1:]  # Update position coordinates
    elif new_data[0] == 1:
        final_data[3:6] = new_data[1:]  # Update velocity components
    elif new_data[0] == 2:
        final_data[6:9] = new_data[1:]  # Update acceleration components
    
    # Check if final_data has any None value
    if None in final_data:
        # Handle the None value here (e.g., set it to 0 or skip the iteration)
        continue

    print(final_data)
    time.sleep(1)
    
    # Rest of your code within the while loop
    # ...
