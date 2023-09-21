import Adafruit_BBIO.GPIO as GPIO
from time import sleep

outPins = ["P9_12", "P8_11", "P8_12", "P8_15", "P8_16", "P8_17", "P8_7", "P8_8", "P8_9", "P8_10"]

def setup_gpio_pins():
    for pin in outPins:
        GPIO.setup(pin, GPIO.OUT)



def rightindicator():
    routPins = ["P8_17","P8_16","P8_15","P8_12","P8_11"]

    while True:

        for pin in routPins:
            GPIO.output(pin,GPIO.HIGH)
            sleep(0.1)
            GPIO.output(pin,GPIO.LOW)

def leftindicator():
    loutPins = ["P8_17","P8_7","P8_8","P8_9","P8_10"]

    while True:

        for pin in loutPins:
            GPIO.output(pin,GPIO.HIGH)
            sleep(0.1)
            GPIO.output(pin,GPIO.LOW)

def idlestate():
    idlePins = ['P8_10', 'P8_9', 'P8_8', 'P8_7', 'P8_17', 'P8_16', 'P8_15', 'P8_12', 'P8_11']
    while True:
        for pin in idlePins:
            GPIO.output(pin, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(pin, GPIO.LOW)
        for pin in reversed(idlePins):
            GPIO.output(pin, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(pin, GPIO.LOW)

def brake():
    brakeoutPins = ["P8_11", "P8_12", "P8_15", "P8_16", "P8_17", "P8_7", "P8_8", "P8_9", "P8_10"]

    for pin in brakeoutPins:
        GPIO.output(pin,GPIO.HIGH)

def hazard():
    rpins = ['P8_17', 'P8_16', 'P8_15', 'P8_12', 'P8_11']
    lpins = ['P8_17', 'P8_7', 'P8_8', 'P8_9', 'P8_10']

    while True:
        for pin1, pin2 in zip(rpins,lpins):
            GPIO.output(pin1, GPIO.HIGH)
            GPIO.output(pin2, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.LOW)

def changestate():
    cgpins =  ['P8_10', 'P8_9', 'P8_8', 'P8_7', 'P8_17', 'P8_16', 'P8_15', 'P8_12', 'P8_11']
    for i in range(5):
        for pin in cgpins:
            GPIO.output(pin, GPIO.HIGH)
        sleep(0.5)  
        for pin in cgpins:
            GPIO.output(pin, GPIO.LOW)
        sleep(0.5)  

def error():
    pin = ["P9_12"]
    for i in range(2):
        GPIO.output(pin[0], GPIO.HIGH)
        sleep(0.5)
        GPIO.output(pin[0], GPIO.LOW)
        sleep(0.5)




try:
    setup_gpio_pins()
    while True:
        
        changestate()

except KeyboardInterrupt:
    GPIO.cleanup()

