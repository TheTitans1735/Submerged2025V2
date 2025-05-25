from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.iodevices  import XboxController
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot,time_it,Stop
ilan=Robot()
from pybricks.parameters import Angle

# async def stop_all():
#     """
#     תכנית לעצירת כל המנועים.

#     """
#     ilan.drive_base.stop()
#     ilan.motor_front.stop()
#     ilan.motor_back.stop()
hub = PrimeHub()

# while True:
#     # Read the tilt values.
#     pitch, roll = hub.imu.tilt()

#     # Print the result.
#     print(pitch, roll)
#     wait(200)
def my_function():
        print("Pitch is greater than 60 degrees!")
        ilan.drive_base.stop()
        ilan.motor_front.stop()
        ilan.motor_back.stop()
async def hi():
    while True:
        pitch, _ = hub.imu.tilt()  # מקבל את ערך ה-pitch במעלות
        if pitch > 60:
            my_function()

        ilan.drive_straight(15,1000)
        hi()
"""
controller = XboxController()

# פונקציה שרצה כאשר הג'ויסטיק הימני מוסט
def do_something():
    print("הג'ויסטיק הימני הופעל!")

while True:
    # קבלת ערכים מהג'ויסטיק הימני (ערכים בין -100 ל-100)
    rx = controller.joystick_right[0]
    ry = controller.joystick_right[1]

    print(f"Right Joystick - X: {rx}, Y: {ry}")

    # אם ערך X של הג'ויסטיק הימני גבוה מ־60, הפעל פונקציה
    if rx > 60:
        do_something()

    wait(100)"""