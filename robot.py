# -*- coding: utf-8 -*-
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port, Stop
from pybricks.tools import wait
from pybricks.parameters import Icon, Color, Button, Direction
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.tools import wait,StopWatch


# # Port A - Right color sensor
# # Port B - Right wheel
# # Port C - Medium motor - front
# # Port D - Medium motor - back
# # Port E - Left color sensor
# # Port F - Left wheel
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    async def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output
    

def time_it(func):
    def wrapper(*args, **kwargs):
        timer = StopWatch()
        result = func(*args, **kwargs)
        run_took = timer.time() / 1000.0
        print(f"run took {run_took} sec")
        return result

    return wrapper


class Robot:
    def __init__(self):
        self.hub = PrimeHub()
        self.left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
        self.right_motor = Motor(Port.B)
        self.motor_front = Motor(Port.C)
        self.motor_back = Motor(Port.D)
        self.drive_base = DriveBase(self.left_motor, self.right_motor,62.4,120) 
        self.left_color_sensor = ColorSensor(Port.E)    
        self.right__color_sensor = ColorSensor(Port.A)       
        self.drive_base.use_gyro(True)
        
    async def buttery_status(self):

        voltage = self.hub.battery.voltage()
        print(f"{voltage=}")

        if voltage> 8000:
            self.hub.light.blink(Color.GREEN,[1000])
        elif voltage> 7500:
            self.hub.light.blink(Color.BLUE,[1000])
        elif voltage> 7000:
            self.hub.light.blink(Color.ORANGE,[1000])
        else:
            self.hub.light.blink(Color.RED,[1000])

    async def run_front_motor(self, speed, angle, wait=True):
        """
        הפעלת המנוע הקדמי לזווית מסוימת במהירות נתונה.
        :param speed: מהירות המנוע.
        :param angle: הזווית שאליה יש להפעיל את המנוע.
        :param wait: האם להמתין עד שהמנוע יגיע לזווית היעד.
        """
        self.motor_front.reset_angle(angle=0)
        await self.motor_front.run_target(speed, target_angle=angle, then=Stop.HOLD, wait=wait)
        
    async def run_back_motor(self, speed, angle, wait=True):
        """
        הפעלת המנוע האחורי לזווית מסוימת במהירות נתונה.
        :param speed: מהירות המנוע.
        :param angle: הזווית שאליה יש להפעיל את המנוע.
        :param wait: האם להמתין עד שהמנוע יגיע לזווית היעד.
        """
        self.motor_back.reset_angle(0)
        await self.motor_back.run_target(speed, angle, then=Stop.HOLD, wait=wait)
    

    async def wait_for_button(self,debug = True):
        """
        המתנה ללחיצה על כפתור.
        :param debug: האם להפעיל מצב דיבוג.
        """
        if not debug:
            return
        self.hub.light.blink(Color.MAGENTA,[1000])
        while not self.hub.buttons.pressed():
            await wait(10)
        self.hub.light.on(Color.BLUE)
    
    async def drive_until_both_on_line(self, threshold=20, speed=200):
        """
        נסיעה עד ששני החיישנים מזהים קו.
        :param threshold: ערך זיהוי הקו.
        :param speed: מהירות הנסיעה.
        """
        self.left_motor.run(speed)
        self.right_motor.run(speed)

        while True:
            left_intensity = await self.left_color_sensor.reflection()        # קריאת חיישן שמאלי
            right_intensity = await self.right__color_sensor.reflection()     # קריאת חיישן ימני            
            print(f"Left: {left_intensity}, Right: {right_intensity}")  # דיבוג

            if left_intensity < threshold: 
                self.left_motor.stop()
            if right_intensity < threshold:
                self.right_motor.stop()

            # אם שני החיישנים מזהים קו (ערכים מתחת לסף)
            if left_intensity < threshold and right_intensity < threshold:
                print("Both sensors detected the line!")
                break

    async def align_to_line(self, threshold=20, speed=100):
        """
        יישור הרובוט כך ששני החיישנים נמצאים על הקו.
        :param threshold: ערך זיהוי הקו.
        :param speed: מהירות התנועה.
        """
        while True:
            left_intensity = self.left_color_sensor.reflection()
            right_intensity = self.right__color_sensor.reflection()

            # אם החיישן השמאלי לא מזהה קו, הנע את המנוע השמאלי
            if left_intensity >= threshold:
                self.left_motor.run(speed)
            else:
                self.left_motor.stop()

            # אם החיישן הימני לא מזהה קו, הנע את המנוע הימני
            if right_intensity >= threshold:
                self.right_motor.run(speed)
            else:
                self.right_motor.stop()

            if left_intensity < threshold and right_intensity < threshold:
                print("Aligned to the line!")
                break
    
    async def drive_straight(
        self, 
        distance_cm, 
        target_speed=1000, 
        stop_at_end=True, 
        timeout_seconds=None, 
        gradual_stop=True, 
        gradtual_start=True,
        kp=1, 
        ki=0, 
        kd=0,
    ):
        acceleration_rate=target_speed/2 if gradtual_start else target_speed
        deceleration_rate=target_speed/2 if gradual_stop else target_speed
        #2025-02-12 rotem remark settings
        self.drive_base.settings(
            straight_speed=target_speed, 
            straight_acceleration=(acceleration_rate, deceleration_rate), 
            turn_rate=None, 
            turn_acceleration=None,
        )

        await self.drive_base.straight(
            distance=distance_cm*10,
            then=Stop.HOLD if stop_at_end else Stop.NONE,
            wait=True,
        )

    async def drive_straight1_with_pid(
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
        # p = סטייה עכשיות 
        # i = מתקן לזווית 0``
        # d = מחזיר למסלול המקורי
        pid = PIDController(kp, ki, kd)
        # Initialize the timer
        timer = StopWatch()
        # Calculate the target angle
        target_distance = distance_cm * 10
        #set the speed according to the distance
        if distance_cm < 0:
            target_speed = -target_speed
            target_distance = -target_distance
        # reset robot angle and distance
        self.drive_base.reset()
        direction = 1 if distance_cm > 0 else -1

        # Drive until the target distance is reached, correct angle using PID
        while True:
            # Calculate the current angle
            current_angle = self.drive_base.angle()
            current_distance = self.drive_base.distance()
            # Calculate the correction
            correction = await pid.compute(0, current_angle)
            # Calculate the speed if gradual start/stop is enabled according to distance
            if abs(current_distance) < target_distance / 2:
                speed = target_speed
                # if gradual_start:
                    # speed = target_speed * abs(current_distance) / (target_distance / 2)
            else:
                speed = target_speed
                if gradual_stop:
                    speed = target_speed * (target_distance - abs(current_distance)) / (target_distance / 2)        
            #set minimum speed
            if abs(speed) < 100:
                speed = 100 * direction 
            print(f"Speed: {speed}, Correction: {correction}, travel: {current_distance}, current_angle: {current_angle}")
            # Set the motor speed
            self.drive_base.drive(speed, correction)
            # Check if the target distance is reached
            if abs(current_distance) >= abs(target_distance):
                break
            # Check if the timeout is reached
            if timeout_seconds is not None and timer.time() > timeout_seconds:
                break  
            await wait(1)
        # Stop the motors
        if stop_at_end:
            self.drive_base.stop()

    # async def arc_turn(self, radius_cm, angle_deg, speed=150):
    #     """
    #     Moves the robot in an arc with a specified radius (in cm) and angle (in degrees).
    #     The radius is measured from the center of the robot to the midpoint between the wheels.
    #     """
    #     # Robot dimensions
    #     wheel_diameter_mm = 57  # Diameter of the wheel in mm
    #     axle_track_cm = 11.2  # Distance between the wheels in cm
    #     gyro_offset_cm = 3.5  # Distance of gyro from wheel center
    #     wheel_circumference_mm = wheel_diameter_mm * 3.14159  # Circumference of the wheel in mm

    #     # Adjust the radius to account for the gyro's offset
    #     effective_radius_cm = radius_cm - gyro_offset_cm

    #     # Calculate the path lengths for the inner and outer wheels
    #     outer_radius_cm = effective_radius_cm + (axle_track_cm / 2)
    #     inner_radius_cm = effective_radius_cm - (axle_track_cm / 2)

    #     # Circumferences of the outer and inner arcs
    #     outer_arc_length_cm = (2 * 3.14159 * outer_radius_cm) * (angle_deg / 360)  # outer circumference * number of rounds
    #     inner_arc_length_cm = (2 * 3.14159 * inner_radius_cm) * (angle_deg / 360)  # inner circumference * number of rounds

    #     # Convert arc lengths to wheel rotations
    #     outer_rotations = (outer_arc_length_cm * 10) / wheel_circumference_mm  # in rotations
    #     inner_rotations = (inner_arc_length_cm * 10) / wheel_circumference_mm  # in rotations

    #     # Calculate speed ratio
    #     if outer_rotations != 0:  # Prevent division by zero
    #         speed_ratio = inner_rotations / outer_rotations
    #     else:
    #         speed_ratio = 0

    #     # Ensure both motors complete their movements together
    #     if radius_cm > 0:  # Turning right
    #         await self.right_motor.run_angle(speed * speed_ratio, inner_rotations * 360, wait=False)  # Inner wheel
    #         await self.left_motor.run_angle(speed, outer_rotations * 360, wait=True)  # Outer wheel
    #     else:  # Turning left
    #         await self.left_motor.run_angle(speed * speed_ratio, inner_rotations * 360, wait=False)  # Inner wheel
    #         await self.right_motor.run_angle(speed, outer_rotations * 360, wait=True)  # Outer wheel

    #     self.left_motor.brake()
    #     self.right_motor.brake()

    #     print(f"Completed arc turn: Radius = {radius_cm} cm, Angle = {angle_deg}° (Adjusted for gyro offset).")
     
    async def turn(self, degrees, speed=150):
        """
        סיבוב הרובוט במספר מעלות מסוים.
        ערכים חיוביים מסובבים בכיוון השעון, ערכים שליליים נגד כיוון השעון.
        :param degrees: מספר המעלות לסיבוב.
        :param speed: מהירות הסיבוב.
        """
        # Reset the built-in gyro sensor to 0 (start angle)
        self.hub.imu.reset_heading(0)

        # Determine the direction of the turn
        if degrees > 0:
            self.left_motor.run(speed)
            self.right_motor.run(-speed)
        else:
            self.left_motor.run(-speed)
            self.right_motor.run(speed)

        # Keep turning until we reach the target angle
        while abs(self.hub.imu.heading() - degrees) > 2:  # Tolerance of 2 degrees
            await wait(0.1) # Wait a little before checking again

        # Stop the motors once we reach the target angle
        self.left_motor.stop()
        self.right_motor.stop()

    async def turn_without_right_wheel(self, degrees, speed=150):
        """
        סיבוב הרובוט במספר מעלות מסוים.
        ערכים חיוביים מסובבים בכיוון השעון, ערכים שליליים נגד כיוון השעון.
        :param degrees: מספר המעלות לסיבוב.
        :param speed: מהירות הסיבוב.
        """
        # Reset the built-in gyro sensor to 0 (start angle)
        self.hub.imu.reset_heading(0)

        # Determine the direction of the turn
        if degrees > 0:
            self.left_motor.run(speed)
            # self.right_motor.run(-speed)
        else:
            self.left_motor.run(-speed)
            # self.right_motor.run(speed)

        # Keep turning until we reach the target angle
        while abs(self.hub.imu.heading() - degrees) > 2:  # Tolerance of 2 degrees
            await wait(0) # Wait a little before checking again

        # Stop the motors once we reach the target angle
        self.left_motor.stop()
        self.right_motor.stop()

    async def arc_turn_with_distance(self, radius_cm, angle_deg, distance_cm, speed=150):
        """
        תנועה בקשת עם רדיוס מסוים (בס"מ), זווית (במעלות) ומרחק (בס"מ).
        הרדיוס נמדד ממרכז הרובוט לנקודת האמצע בין הגלגלים.
        :param radius_cm: רדיוס הקשת בס"מ.
        :param angle_deg: זווית הקשת במעלות.
        :param distance_cm: מרחק הנסיעה בס"מ.
        :param speed: מהירות הנסיעה.
        """
        # Calculate the total arc length for the given distance
        total_arc_length_cm = (2 * 3.14159 * radius_cm) * (angle_deg / 360)
        
        # Calculate the number of segments to divide the arc into
        num_segments = int(distance_cm / total_arc_length_cm)
        
        for _ in range(num_segments):
            await self.arc_turn(radius_cm, angle_deg, speed)
        
        # Calculate the remaining distance and turn
        remaining_distance_cm = distance_cm % total_arc_length_cm
        remaining_angle_deg = (remaining_distance_cm / total_arc_length_cm) * angle_deg
        
        await self.arc_turn(radius_cm, remaining_angle_deg, speed)


    async def drive_with_turn(self, distance_cm, turn_rate, speed=150):
        """
        נסיעה קדימה תוך כדי סיבוב בקצב מסוים.
        :param distance_cm: המרחק לנסיעה בסנטימטרים.
        :param turn_rate: קצב הסיבוב במעלות לשנייה.
        :param speed: מהירות הנסיעה במעלות לשנייה.
        """
        # Calculate the target distance in millimeters
        target_distance_mm = distance_cm * 10
        
        # Reset the drive base
        self.drive_base.reset()
        
        # Drive while turning until the target distance is reached
        while abs(self.drive_base.distance()) < abs(target_distance_mm):
            self.drive_base.drive(speed, turn_rate)
            await wait(10)
        
        # Stop the drive base
        self.drive_base.stop()
