from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot,time_it
from pybricks.parameters import Axis


hub = PrimeHub()
#hub.imu.ready()
left=-1
right=1
hub.imu.reset_heading(0)
left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.B)

#        self.motor_front = Motor(Port.C)
#        self.motor_back = Motor(Port.D)
drive_base = DriveBase(left_motor, right_motor,62.4,120) 
drive_base.use_gyro(True)
drive_base.settings()
print("1")
# drive_base.distance_control.pid(1,0,0)


# print(f"{hub.imu.heading()}")
# wait(3000)
def drive_1():
    drive_base.straight(750)
    wait(1000)
    drive_base.turn(-90)
    wait(1000)
    drive_base.straight(480)
    wait(1000)
    drive_base.turn(-90)
    wait(1000)
    drive_base.straight(410)
    wait(1000)
    drive_base.turn(90)
    wait(1000)
    drive_base.straight(600)
    wait(100)
    drive_base.turn(45)
    wait(100)
    drive_base.straight(350)
    wait(100)
    drive_base.turn(45)
    wait(100)
    drive_base.straight(400)

def drive_2():
    drive_base.straight(600)
    wait(100)
    drive_base.turn(-45)
    wait(100)
    drive_base.straight(140)
    wait(100)
    drive_base.turn(-30)
    wait(100)
    drive_base.straight(300)
    wait(100)
    drive_base.turn(-10)
    wait(100)
    drive_base.straight(800)
    exit
    wait(100)
    drive_base.turn(90)
    wait(100)
    drive_base.straight(154)

def drive_3():
    drive_base.turn(23)
    wait(100)
    drive_base.straight(480)
    wait(100)
    drive_base.turn(65)
    wait(100)
    drive_base.straight(200)
    drive_base.straight(-140)
    drive_base.turn(135)
    drive_base.straight(-270)
    drive_base.turn(-40)
    drive_base.straight(-180)
    drive_base.straight(10)
    drive_base.straight(70)
    drive_base.turn(35)
    drive_base.straight(600)
drive_3()
# drive_base.curve(550,90)
# print(f"{hub.imu.heading()}")
# wait(3000)
# drive_base.turn(90)
# print(f"{hub.imu.heading()}")
# wait(3000)
# drive_base.turn(90)
# print(f"{hub.imu.heading()}")
# wait(3000)
# while True:
#     print(f"{hub.imu.heading()}")
#     wait(1000)

# def turn( degrees, speed=150):
#         """
#         סיבוב הרובוט במספר מעלות מסוים.
#         ערכים חיוביים מסובבים בכיוון השעון, ערכים שליליים נגד כיוון השעון.
#         :param degrees: מספר המעלות לסיבוב.
#         :param speed: מהירות הסיבוב.
#         """
#         # Reset the built-in gyro sensor to 0 (start angle)
#         hub.imu.reset_heading(0)

#         # Determine the direction of the turn
#         if degrees > 0:
#             left_motor.run(speed)
#             right_motor.run(-speed)
#         else:
#             left_motor.run(-speed)
#             right_motor.run(speed)

#         # Keep turning until we reach the target angle
#         abs(hub.imu.heading() - degrees) > 2:  # Tolerance of 2 degrees
#         wait(0) # Wait a little before checking again

#         # Stop the motors once we reach the target angle
#         left_motor.stop()
#         right_motor.stop()
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output


    def drive_straight(
        self, 
        distance_cm, 
        target_speed=300, 
        stop_at_end=True, 
        timeout_seconds=None, 
        gradual_stop=True, 
        gradtual_start=True,
        kp=1, 
        ki=0, 
        kd=0,
    ):
        """
        נסיעה ישרה באמצעות בקרת PID על בסיס זווית הבסיס.
        :param distance_cm: מרחק ב-ס"מ.
        :param target_speed: מהירות ב-מעלות לשנייה.
        :param stop_at_end: האם לעצור את המנועים בסיום.
        :param timeout_seconds: זמן מקסימלי לנסיעה בשניות.
        :param gradual_stop: האם לבצע עצירה הדרגתית.
        :param gradual_start: האם לבצע התחלה הדרגתית.
        :param kp: מקדם פרופורציונלי.
        :param ki: מקדם אינטגרלי.
        :param kd: מקדם נגזר.
        """
        # Initialize PID controller
        # # p = סטייה עכשיות 
        # # i = מתקן לזווית 0``
        # # d = מחזיר למסלול המקורי
        # pid = PIDController(kp, ki, kd)
        # # Initialize the timer
        # timer = StopWatch()
        # # Calculate the target angle
        # target_distance = distance_cm * 10
        # #set the speed according to the distance
        # if distance_cm < 0:
        #     target_speed = -target_speed
        #     target_distance = -target_distance
        # # reset robot angle and distance
        # self.drive_base.reset()
        # direction = 1 if distance_cm > 0 else -1

        # # Drive until the target distance is reached, correct angle using PID
        # while True:
        #     # Calculate the current angle
        #     current_angle = self.drive_base.angle()
        #     current_distance = self.drive_base.distance()
        #     # Calculate the correction
        #     # correction = await pid.compute(0, current_angle)
        #     # Calculate the speed if gradual start/stop is enabled according to distance
        #     if abs(current_distance) < target_distance / 2:
        #         speed = target_speed
        #         if gradual_start:
        #             speed = target_speed * abs(current_distance) / (target_distance / 2)
        #     else:
        #         speed = target_speed
        #         if gradual_stop:
        #             speed = target_speed * (target_distance - abs(current_distance)) / (target_distance / 2)        
        #     #set minimum speed
        #     if abs(speed) < 100:
        #         speed = 100 * direction 
        #     print(f"Speed: {speed}, Correction: {correction}, travel: {current_distance}, current_angle: {current_angle}")
        #     # Set the motor speed
        #     self.drive_base.drive(speed, correction)
        #     # Check if the target distance is reached
        #     if abs(current_distance) >= abs(target_distance):
        #         break
        #     # Check if the timeout is reached
        #     if timeout_seconds is not None and timer.time() > timeout_seconds:
        #         break  
        #     await wait(1)
        # # Stop the motors
        # if stop_at_end:
        #     self.drive_base.stop()
