from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor,ForceSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Stop, Icon, Color, Button, Direction
from pybricks.tools import wait, StopWatch
from pybricks.iodevices import XboxController
class Robot:
    def __init__(self):
        self.hub = PrimeHub()
        self.left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.D)
        self.motor_front = Motor(Port.E)
        self.motor_back = Motor(Port.A)
        self.drive_base = DriveBase(self.left_motor, self.right_motor, 62.4, 120)
        self.xbox = XboxController()
        self.drive_base.use_gyro(True)

    def run(self):
        while True:
            pressed = self.xbox.buttons.pressed()

            if Button.LEFT in pressed:
                # פנייה שמאלה מתמשכת
                self.drive_base.drive(0, -100)  # (מהירות קדימה=0, מהירות זווית=-100)
            elif Button.RIGHT in pressed:
                # פנייה ימינה מתמשכת
                self.drive_base.drive(0, 100)
            elif Button.DOWN in pressed:
                # עצירה
                self.drive_base.drive(-250 , 0)  
            elif Button.UP in pressed:
                # נסיעה קדימה מתמשכת
                self.drive_base.drive(250, 0)
            else:
                # לא לוחצים - עוצרים
                self.drive_base.stop()

Robot().run()