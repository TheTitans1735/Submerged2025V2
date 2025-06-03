from pybricks.pupdevices import ColorSensor, ForceSensor, Motor
from pybricks.hubs import PrimeHub
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Direction
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Icon, Color, Button
from pybricks.parameters import Icon, Color, Button, Direction
from pybricks.parameters import Port
from pybricks.tools import wait


if ForceSensor(Port.E).pressed(True):
    print("Waiting for force sensor to be pressed...")


# print_force_sensor()