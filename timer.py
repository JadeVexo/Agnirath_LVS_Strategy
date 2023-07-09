import time

class Timer:
    def __init__(self, threshold_crossing_time):
        self.threshold_crossing_time = threshold_crossing_time
        self.start_time = None

    def start(self):
        self.start_time = time.time()

    def elapsed(self):
        return time.time() - self.start_time

    def reset(self):
        self.start_time = None

def get_temperature():
    # Replace this with your actual code to retrieve temperature and humidity values
    temperature = 60  # Placeholder temperature value
    return temperature


def main_loop():
    temperature_threshold = 30  # Temperature threshold to check
    threshold_crossing_time = 3  # Number of seconds to check for threshold crossing
    
    temperature_timer = Timer(threshold_crossing_time)

    temperature= get_temperature()

    while True:
        

        if temperature > temperature_threshold:
            if temperature_timer.start_time is None:
                temperature_timer.start()

            if temperature_timer.elapsed() >= threshold_crossing_time:
                print(f"Temperature threshold crossed for {threshold_crossing_time} seconds: {temperature}°C")
                temperature = 0
        else:
            temperature_timer.reset()
            temperature = 60

        # print(temperature)
        time.sleep(0.1)

main_loop()
            
