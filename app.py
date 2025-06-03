from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor, ForceSensor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot,time_it,Stop, over_roll
# from pynput import keyboard
# for ilan
# check change 
# change 4
ilan=Robot()
# # Port A - Right color sensor
# # Port B - Right wheel
# # Port C - Medium motor - front
# # Port D - Medium motor - back
# # Port E - Left color sensor
# # Port F - Left wheel

# hub = PrimeHub()
# left_motor = Motor(Port.F, Direction.COUNTERCLOCKWISE)
# right_motor = Motor(Port.B)
# motor_front = Motor(Port.C)
# motor_back = Motor(Port.D)
# drive_base = DriveBase(left_motor,right_motor,57,10) 


async def drive(): 
    """
    תכנית לנסיעה קדימה.
    """  
    await ilan.drive_base.drive(750, 0)
async def reverse_drive():
    """
    תכנית לנסיעה אחורה.
    """
    await ilan.drive_base.drive(-750, 0)

async def turn_left():
    """
    תכנית לסיבוב שמאלה.
    """
    await ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    """
    תכנית לסיבוב ימינה.
    """
    await ilan.drive_base.turn(360, wait=False)

async def front_motor():
    """
    תכנית להפעלת מנוע קדמי.
    """
    await ilan.motor_front.dc(50)

async def back_motor():
    """
    תכנית להפעלת מנוע אחורי.
    """
    await ilan.motor_back.dc(100)

async def front_motor_reverse():
    """
    תכנית להפעלת מנוע קדמי לאחור.
    """
    await ilan.motor_front.dc(-50)

async def back_motor_reverse():
    """
    תכנית להפעלת מנוע אחורי לאחור.
    """
    await ilan.motor_back.dc(-100)

async def stop_all():
    """
    תכנית לעצירת כל המנועים.
    """
    ilan.drive_base.stop()
    ilan.motor_back.stop()
    ilan.motor_front.stop()


async def turn():
    """
    תכנית לסיבוב.
    """
    await ilan.turn(90,150)
    await ilan.wait_for_button(True)
    await wait(1000)
    await ilan.turn(-90, 150)

async def prepare_whale_motor():
    """
    תכנית להכנת המנוע למשימת הלוויתן.
    """
    await wait(1000)
    await ilan.run_back_motor(200,-181)


@time_it
async def whale():
    """
    ביצוע משימת הלוויתן.
    """
    debug= False
    ilan.drive_base.reset()
    await multitask(
        ilan.drive_straight(28,850),
        prepare_whale_motor()
    )
    await ilan.wait_for_button(debug)    
    await ilan.turn(-28)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(40,500)
    await ilan.wait_for_button(debug)
    # פניה אל הלוייתן
    await ilan.turn(70)
    # נסיעה אל הלוייתן ללא האטה כדי להפיל את החסילונים
    await ilan.drive_straight(29,800,gradual_stop=False)
    await wait(500)
    await multitask(
        ilan.drive_straight(-32,700), 
        ilan.motor_back.run_time(-250,1500)
    )
    await ilan.turn(120)
    await ilan.run_back_motor(100,165)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-27,900)
    await ilan.run_back_motor(150,-60)  
    await ilan.drive_straight(19)
    await ilan.turn(-15)

async def sonar():
    """
    פונקציה המבצעת את משימת ההאכילו את הלוויתן ותגלית סונאר.
    """
    await ilan.drive_straight(-30,300)
    await ilan.turn(80,200)
    await ilan.drive_straight(-13)
    await ilan.run_back_motor(150,-210)
    await ilan.drive_straight(-53,10)
    await ilan.run_back_motor(300,-90)

async def pick_up():
    """
    מבצע את משימות מפגש בלתי צפוי, דגימת פלנקטון ואיסוף
    """
    await ilan.drive_straight(41 ,670,gradual_stop=False)
    await ilan.drive_straight(-11,500)
    await ilan.motor_front.run_angle(700,250)
    await ilan.turn(42,100 )
    await wait(100)
    await ilan.drive_straight(42)
    await ilan.turn(35)
    await ilan.drive_straight(12)
    await ilan.motor_front.run_angle(700,-250)
    ilan.drive_base.settings(-700,None,None,None)
    await ilan.drive_base.curve(-400,60,Stop.NONE)
    await ilan.drive_straight(-40,700)

@time_it
async def crabs():
    """
    מבצע את משימת ספינת מחקר בית גידול מלאכותי ותיבה"
    """
    await ilan.drive_straight(-107,800)
    await ilan.motor_back.run_time(200,2500)
    await ilan.drive_straight(12,120)
    await ilan.turn(5)
    await multitask(ilan.drive_straight(25,650), ilan.run_back_motor(200, -100))
    await ilan.turn(-31,100)
    await ilan.drive_straight(-35,750)
    await ilan.turn(24,50)
    await ilan.drive_straight(-47,750)
    await ilan.turn(25,550)
    await ilan.drive_straight(-45,1000)

@time_it
async def green():
    """
    מבצע את משימת אוצרו של הקראקן, הרימו תורן, שונית האלמוגים ותליית הצוללן"
    """
    Debug = False
    await ilan.turn(23,100)
    await ilan.drive_straight(48, 700)
    await ilan.turn(65,100)
    await ilan.wait_for_button(Debug)
    await wait(100)
    await ilan.drive_straight(23, 700)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(-12, 700)
    await ilan.turn(130, 150)
    await ilan.drive_straight(-26, 700)
    await ilan.wait_for_button(Debug)
    await ilan.turn(-40,100)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(-18, 700,gradual_stop=True)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(3, 700)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(-700,1500)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(500,1500)
    await ilan.drive_straight(6.5,700)
    await ilan.turn(45)
    await ilan.drive_straight(77,700)

@time_it
async def coral():
    """"
    מבצע את משימת העמידו את האלמוגים
    """
    await ilan.drive_straight(30,500)
    await ilan.drive_straight(-19)



@time_it
async def massive():
    """
    מבצע את משימת משתלת אלמוגים, כריש, צוללן ואיסוף"
    """
    debug= False
    await ilan.drive_straight(10,700)
    await ilan.turn(90)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(100,175)
    await ilan.wait_for_button(debug)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(14,500)
    await ilan.wait_for_button(debug)
    await ilan.wait_for_button(debug)
    await ilan.motor_back.run_time(-200,1000)
    await ilan.motor_front.run_angle(200,-400)
    await wait(500) 
    await ilan.wait_for_button(debug)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,5)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(400,500)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-10,700)
    await ilan.wait_for_button(debug)
    await ilan.turn(30)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-70,700)
    # await ilan.wait_for_button(debug)


async def test():
        ilan.drive_base.drive(120, 0)


async def battery_check():
    pass


# async def monitor_force():
#     while True:
#         thouch, press = ilan.()
#         if abs(roll) > 50:
#             ilan.drive_base.stop()
#             ilan.motor_back.stop()
#             ilan.motor_front.stop()
#             raise over_roll(f"Roll exceeded: {roll}")
#         await wait(50)

        
async def monitor_roll():
    roll_exceeded = False
    while True:
        try:
            pitch, roll = ilan.hub.imu.tilt()
            if abs(roll) > 50:
                if not roll_exceeded:
                    print(f"Roll exceeded: {roll}")
                    roll_exceeded = True
                ilan.drive_base.stop()
                ilan.motor_back.stop()
                ilan.motor_front.stop()
                await stop_all()
                await wait(100)
                raise over_roll(f"Roll exceeded: {roll}")  # <--- הוספה כאן
            else:
                roll_exceeded = False
        except Exception as e:
            print("Error in monitor_roll:", e)
        await wait(50)
"""
    פונקציה המבצעת את כל התכניות
"""
async def main_loop():
    runs = [
        # --- משימות עיקריות ---
        ("0", battery_check, Icon.TRIANGLE_UP),
        ("1", massive,       Icon.LEFT),
        ("2", green,         Icon.FALSE),
        ("3", crabs,         Icon.HAPPY),
        ("4", pick_up,       Icon.SAD),
        ("5", coral,         Icon.PAUSE),
        ("7", whale,         Icon.FULL),
        ("8", sonar,         Icon.HEART),

        # --- פעולות נהיגה ---
        (" ", drive,        Icon.ARROW_LEFT),
        (" ", reverse_drive,Icon.ARROW_RIGHT),
        (" ", turn_left,    Icon.ARROW_LEFT_DOWN),
        (" ", turn_right,   Icon.ARROW_LEFT_UP),
        (" ", turn,         Icon.CLOCKWISE),

        # --- שליטה במנועים ---
        ("1", front_motor),
        ("2", back_motor),
        ("3", front_motor_reverse),
        ("4", back_motor_reverse),

        # --- בדיקות ופיתוח ---
        ("T", test, Icon.TRIANGLE_DOWN),
    ]





    current_run = 0
    await ilan.buttery_status()
    buttery_status_timer = StopWatch()
    while True:
        if buttery_status_timer.time() > 10000:
            await ilan.buttery_status()
            buttery_status_timer.reset()

        try:
            pressed = ilan.hub.buttons.pressed()

            if Button.LEFT in pressed:
                current_run += 1
                if current_run >= len(runs):
                    current_run = 0
                if len(runs[current_run]) == 2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])

            elif Button.RIGHT in pressed:
                current_run -= 1
                if current_run < 0:
                    current_run = len(runs) - 1
                if len(runs[current_run]) == 2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])

            elif Button.BLUETOOTH in pressed:
                try:
                    await runs[current_run][1]()
                    current_run = (current_run + 1) % len(runs)
                except over_roll as e:
                    print(f"Roll too high! {e}")
                    await stop_all()
                    await wait(700)
                if len(runs[current_run]) == 2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])
            else:
                await stop_all()

        except Exception as e:
            print(e)
            raise e
        finally:
            await wait(150)
async def main():
    await multitask(monitor_roll(),main_loop())
run_task(main())
