# from pybricks.pupdevices import ForceSensor
# from pybricks.parameters import Port
# from pybricks.tools import wait

# # Initialize the sensor.
# buttonB = ForceSensor(Port.B)
# buttonA = ForceSensor(Port.A)
# previousTA =False
# previousTB =False
# while True:
#     # Read all the information we can get from this sensor.
#     forceA = buttonA.force()
#     distA = buttonA.distance()
#     pressA = buttonA.pressed()
#     touchA = buttonA.touched()

#     forceB = buttonB.force()
#     distB = buttonB.distance()
#     pressB = buttonB.pressed()
#     touchB = buttonB.touched()
#     # Print the values
#     print("FA", forceA, "D:", distA, "P:", pressA, "T:", touchA,"******FB", forceB, "D:", distB, "P:", pressB, "T:", touchB)
#     if touchA != previousTA:
#         wait(10000)
#     if touchB != previousTB:
#         wait(10000)
#     previousTA = touchA
#     previousTB = touchB
# #     ilan.motor_back.dc(50)
#     # Push the sensor button see what happens to the values.

#     # Wait some time so we can read what is printed.
#     wait(500)

from pybricks.pupdevices import UltrasonicSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the sensor.
eyes = UltrasonicSensor(Port.A)

while True:
    # Print the measured distance.
    print(eyes.distance())

    # If an object is detected closer than 500mm:
    if eyes.distance() < 500:
        # Turn the lights on.
        eyes.lights.on(100)
    else:
        # Turn the lights off.
        eyes.lights.off()

    # Wait some time so we can read what is printed.
    wait(100)