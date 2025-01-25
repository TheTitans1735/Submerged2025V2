from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot, time_it
# from pynput import keyboard
# for gebetta
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
    נסיעה קדימה.
    """
    ilan.drive_base.drive(151,0)


async def reverse_drive():
    """
    נסיעה אחורה.
    """
    ilan.drive_base.drive(-750, 0)

async def turn_left():
    """
    סיבוב שמאלה.
    """
    ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    """
    סיבוב ימינה.
    """
    ilan.drive_base.turn(360, wait=False)

async def front_motor():
    """
    הפעלת המנוע הקדמי.
    """
    ilan.motor_front.dc(100)

async def back_motor():
    """
    הפעלת המנוע האחורי.
    """
    ilan.motor_back.dc(100)

async def front_motor_reverse():
    """
    הפעלת המנוע הקדמי בכיוון הפוך.
    """
    ilan.motor_front.dc(-50)
    
async def back_motor_reverse():
    """
    הפעלת המנוע האחורי בכיוון הפוך.
    """
    ilan.motor_back.dc(-100)

async def stop_all():
    """
    עצירת כל המנועים.
    """
    ilan.drive_base.stop()
    ilan.motor_back.stop()
    ilan.motor_front.stop()

async def nigg():
    """
    ביצוע משימה מסוימת.
    """
    await ilan.drive_base.straight(320.50)
    await wait(1000)
    ilan.motor_back.dc(80)
    await wait(1000)
    ilan.motor_back.dc(-10)
    await ilan.drive_base.straight(20)
    await wait(1000)
    await ilan.drive_base.straight(-350)

async def turn():
    """
    סיבוב הרובוט במספר מעלות מסוים.
    """
    await ilan.turn(90,150)
    await ilan.wait_for_button(True)
    await wait(1000) 
    await ilan.turn(-90, 150)

async def prepare_whale_motor():
    """
    הכנת המנוע האחורי למשימה.
    """
    await ilan.run_back_motor(100,100)
    # await wait(1000)
    await ilan.run_back_motor(200,-290)

@time_it
async def whale():
    """
    ביצוע משימת הלוויתן.
    """
    pid = {"kp":1, "ki":0, "kd": 0}
    ilan.drive_base.reset()
    await ilan.drive_straight(19)
    await ilan.turn(-15)
    await multitask(ilan.drive_straight(50,200, **pid), prepare_whale_motor())
    await ilan.wait_for_button(debug=False)
    await ilan.turn(55)
    await ilan.drive_straight(19,190,gradual_stop=False, **pid)
    await wait(1000)
    await ilan.drive_straight(-2,150, **pid)
    # await ilan.wait_for_button()
    await ilan.drive_straight(2,200,gradual_stop=False, **pid)
    # await ilan.wait_for_button()
    await multitask(ilan.drive_straight(-30,100, **pid), ilan.motor_back.run_angle(250,-290))
    await ilan.turn(120)
    # await ilan.wait_for_button()
    await ilan.motor_back.run_angle(250,90)
    await ilan.motor_back.run_angle(150  ,135)
    await ilan.drive_straight(-20,150,**pid,)
    # await ilan.turn(14)
    # await ilan.drive_straight(8)
    await multitask(ilan.drive_straight(-9, **pid))
    await ilan.run_back_motor(200, -105)        
    await ilan.drive_straight(19)
    # await ilan.wait_for_button(
    # )
    await ilan.turn(-15)
    # await ilan.wait_for_button()
    await multitask(ilan.drive_straight(55,700,gradual_stop=False ,**pid), ilan.run_back_motor(800,-290))
    await ilan.run_back_motor(200,290)

async def sonar():
    """
    ביצוע משימת הסונאר.
    """
    await ilan.drive_straight(-30,300)
    await ilan.turn(90)
    await ilan.drive_straight(13)
    await ilan.run_back_motor(50,-120)
    await ilan.turn(-5)
    await ilan.drive_straight(-53)
    await ilan.run_back_motor(300,-90)

@time_it
async def coral():
    """
    ביצוע משימת האלמוגים.
    """
    await ilan.drive_straight(8,100)
    await ilan.drive_straight(-5,500,gradual_stop=False)


@time_it
async def banana():
    """
    ביצוע משימת הבננה.
    """
    pid = {"kp":1, "ki": 0, "kd": 0.0}
    await ilan.drive_straight(44,200, gradual_stop= False, **pid)   #נסיעה ישר
    await ilan.drive_straight(-13,300, **pid)                       #נסיעה אחורה
    await ilan.run_front_motor(310,300)                             #הזזת המנוע האחורי 
    await ilan.turn(40)                                             #מבצע פנייה
    await ilan.drive_straight(37,230,**pid)                         #נסיעה ישר
    await ilan.turn(35)                                             #מבצע פנייה
    await ilan.drive_straight(17,260,**pid)                         #נסיעה ישר
    await ilan.motor_front.run_angle(200,-300)                      #הזזת מנוע אחורי 
    await ilan.drive_straight(-20, 450)                             #נסיעה אחורה
    await ilan.turn(-60)                                            #מבצע פנייה
    await ilan.drive_straight(-50, 500, gradual_start=False,        #נסיעה אחורה
     gradual_stop=False)   
 

@time_it
async def crabs():
    """
    ביצוע משימת הסרטנים.
    """
    await ilan.drive_straight(60)
    await ilan.turn(5)
    await ilan.drive_straight(-22)
    await ilan.run_back_motor(200,150)
    await ilan.drive_straight(10)
    await ilan.turn(90)
    await ilan.drive_straight(20)
    await ilan.turn(-90)
    await ilan.drive_straight(-90)

@time_it
async def green():
    """
    ביצוע משימת הירוק.
    """
    pid = {"kp": 1, "ki": 0, "kd": 0.0}
    await ilan.turn(20,200)
    await ilan.drive_straight(50,200, **pid)
    await ilan.turn(66,200)
    await ilan.wait_for_button()
    await ilan.drive_straight(13,200, **pid)
    await ilan.run_front_motor(200,-80)
    await ilan.drive_straight(1.5,200, **pid)
    await ilan.run_front_motor(200,20)

@time_it
async def massive():
    """
    ביצוע משימת המסיבי.
    """
    debug= False
    pid = {"kp": 1, "ki": 0, "kd": 0.0}
    await ilan.drive_straight(52,300, **pid)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(150,175)
    await ilan.wait_for_button(debug)
    # await ilan.turn(-4,200)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(14,300, **pid)
    await ilan.wait_for_button(debug)
    # await ilan.drive_straight(-1.5,200)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(200,-55)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,-400)
    await wait(500) 
    await ilan.wait_for_button(debug)
    #await ilan.turn(10,200)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,5)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(400,650)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-10,200, **pid)
    await ilan.wait_for_button(debug)
    await ilan.turn(30)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-80,500, **pid)
    await ilan.wait_for_button(debug)
    # test7

@time_it
async def test():
    """
    בדיקת נסיעה ישרה עם בקרת PID.
    """
    pid = {"kp":1, "ki":0.1, "kd": 0.01}
    await ilan.drive_straight(60,200, gradual_stop= False, **pid)


async def main():
    """
    התוכנית הראשית.
    """
    runs = [
        ("7", whale, Icon.SAD),
        ("5", drive, Icon.ARROW_LEFT),
        ("6", reverse_drive, Icon.ARROW_RIGHT),
        ("7", turn_left, Icon.ARROW_LEFT_DOWN),
        ("8", turn_right, Icon.ARROW_LEFT_UP),
        ("1", front_motor),
        ("2", back_motor),
        ("3", front_motor_reverse),
        ("4", back_motor_reverse),
        ("5", nigg, Icon.CIRCLE),
        ("6", turn, Icon.CLOCKWISE),
        ("crabs", crabs, Icon.HAPPY),
        ("2", banana, Icon.EYE_LEFT_BROW),
        ("T", test),
        ("8", sonar,Icon.HEART),
        ("1", massive, Icon.LEFT),
        ("2", green, Icon.FALSE),
        ("3", coral ,Icon.CIRCLE)

        # ("9", play_sound)
    ]
    current_run = 0
    print("current", ilan.hub.battery.current(), "voltage", ilan.hub.battery.voltage())
    
    while True:
        try:
            "מעביר לתוכנית הבאה"
            if (Button.LEFT in ilan.hub.buttons.pressed()):
                current_run += 1
                
                if current_run >= len(runs):
                    current_run = 0
                # הצגת האייקון או האות המתאים למשימה הנוכחית
                if len(runs[current_run]) == 2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])

            # כאשר נפעיל את המשימה הנוכחית
            elif (Button.RIGHT in ilan.hub.buttons.pressed()):
                await runs[current_run][1]()

        #   כאשר נלחץ על כפתור בלוטוס  נפעיל תוכנית הניסיון (TEST)
            elif (Button.BLUETOOTH in ilan.hub.buttons.pressed()):
                await test()
            else:
                await stop_all()
        except Exception as e:
            print(e)
            raise e
        finally:
            await wait(100)


run_task(main())




