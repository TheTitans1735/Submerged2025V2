from pybricks.pupdevices import ColorSensor, ForceSensor, Motor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Icon, Color, Button
from pybricks.parameters import Icon, Color, Button, Direction
from pybricks.parameters import Port
from pybricks.tools import wait

class Robot:
    def __init__(self):
        self.hub = PrimeHub()
        self.left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.E)
        self.motor_front = Motor(Port.F)
        self.motor_back = Motor(Port.D)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, 62.4, 120)
        self.color_sensor = ColorSensor(Port.B)
        self.forcesensor = ForceSensor(Port.C)        
        self.drive_base.use_gyro(True)

    async def buttery_status(self):
        DriveBase.drive(DriveBase,100,0)

# # Initialize the sensor.
# sensor = ColorSensor(Port.A)

# while True:
#     # Read the color and reflection
#     color = sensor.color()
#     reflection = sensor.reflection()

#     # Print the measured color and reflection.
#     print(color, reflection)

#     # Move the sensor around and see how
#     # well you can detect colors.

#     # Wait so we can read the value.
#     wait(100)
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