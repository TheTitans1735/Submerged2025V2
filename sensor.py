from pybricks.pupdevices import ColorSensor, ForceSensor, Motor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Icon, Color, Button
from pybricks.parameters import Icon, Color, Button, Direction
from pybricks.parameters import Port
from pybricks.tools import wait


"""הדפסת ערכי חיישן כוח"""

hub = PrimeHub()
forcesensor = ForceSensor(Port.D)
while True:
    print(f"press {forcesensor.pressed()}-----------------touch {forcesensor.touched()}")
    wait(1000)

"""הפעלת פונקציה להדפסת ערכי חיישן כוח"""
