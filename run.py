from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot
# from pynput import keyboard

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
    ilan.drive_base.drive(151,0)


async def reverse_drive():
    ilan.drive_base.drive(-750, 0)

async def turn_left():
    await ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    await ilan.drive_base.turn(360, wait=False)

async def front_motor():
    ilan.motor_front.dc(5000)

async def back_motor():
    ilan.motor_back.dc(4000)

async def front_motor_reverse():
    ilan.motor_front.dc(-50)
    
async def back_motor_reverse():
    ilan.motor_back.dc(-500)

async def stop_all():
    ilan.drive_base.stop()
    ilan.motor_back.stop()
    ilan.motor_front.stop()

async def nigg():
    await ilan.drive_base.straight(320.50)
    await wait(1000)
    ilan.motor_back.dc(80)
    await wait(1000)
    ilan.motor_back.dc(-10)
    await ilan.drive_base.straight(20)
    await wait(1000)
    await ilan.drive_base.straight(-350)

async def turn():
    await ilan.turn(90,150)
    await ilan.wait_for_button(True)
    await wait(1000) 
    await ilan.turn(-90, 150)

async def prepare_whale_motor():
    await ilan.run_back_motor(100,100)
    # await wait(1000)
    await ilan.run_back_motor(200,-290)


async def whale():
    pid = {"kp":1, "ki":0.1, "kd": 0.01}
    await multitask(ilan.drive_straight(62,200, **pid), prepare_whale_motor())
    await ilan.wait_for_button(debug=False)
    await ilan.turn(73)
    await ilan.wait_for_button(debug=True)
    await ilan.drive_straight(30,150, **pid)
    await wait(1000)
    await ilan.drive_straight(-2,150, **pid)
    await ilan.wait_for_button()
    await ilan.drive_straight(4, **pid)
    await multitask(ilan.drive_straight(-20,100, **pid), ilan.motor_back.run_angle(250,-290))
    await ilan.turn(100)
    await ilan.motor_back.run_angle(250,90)
    await ilan.motor_back.run_angle(80,120)
    await ilan.drive_straight(-20,150,**pid,)
    # await ilan.turn(14)
    # await ilan.drive_straight(8)
    await multitask(ilan.drive_straight(-9, **pid))
    await ilan.run_back_motor(200, -120)

async def sonar():
    await ilan.drive_straight(-30,300)
    await ilan.turn(90)
    await ilan.drive_straight(13)
    await ilan.run_back_motor(50,-120)
    await ilan.turn(-5)
    await ilan.drive_straight(-53)
    await ilan.run_back_motor(300,-90)

async def banana():
    pid = {"kp":1.2, "ki":0.1, "kd": 0.01}
    await ilan.motor_back.run_angle(100,1)
    await ilan.drive_straight(42,200, gradual_stop= False, **pid)
    await ilan.drive_straight(-18,300, **pid)
    await ilan.run_front_motor(310,300)
    await ilan.turn(32,170)
    await ilan.drive_straight(33,230,**pid)
    await ilan.turn(36,170)
    await ilan.drive_straight(7.5,260,**pid)
    await ilan.wait_for_button()
    # await ilan.turn(28,170)
    # await ilan.wait_for_button()
    # await ilan.run_front_motor(320,-300)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(-10,260, **pid)
    # await ilan.wait_for_button()
    # await ilan.turn(-133,170)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(-7,330 ,**pid)
    # await ilan.wait_for_button()
    # await ilan.run_back_motor(200,360)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(-5,370, **pid)

async def crabs():
    await ilan.drive_straight(60)
    await ilan.turn(5)
    await ilan.drive_straight(-22)
    await ilan.run_back_motor(200,150)
    await ilan.drive_straight(10)
    await ilan.turn(90)
    await ilan.drive_straight(20)
    await ilan.turn(-90)
    await ilan.drive_straight(-90)

async def massive():
    pid = {"kp": 1.32, "ki": 0.1, "kd": 0.01}
    await ilan.drive_straight(52,300, **pid)
    await ilan.run_back_motor(200,160)
    await ilan.turn(-4,200)
    await ilan.drive_straight(18.5,300, gradual_start=False, **pid)
    await ilan.drive_straight(-2.5,300)
    await ilan.run_back_motor(200,-50)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(200,-400)
    await ilan.motor_front.run_angle(200,5)
    await ilan.motor_front.run_angle(400,650)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.drive_straight(-10,200, **pid)
    await ilan.turn(30)
    await ilan.drive_straight(-40,200, **pid)

    # await ilan.motor_front.run_angle(200,-400)
    # await ilan.motor_front.run_angle(200,200)
    # await ilan.drive_straight(10,200)
    # await ilan.turn(30)
    # await ilan.drive_straight(-30)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(1.5,200)
    # await ilan.wait_for_button()
    # await ilan.motor_front.run_angle(300,1000)
    # await ilan.wait_for_button()
    # await ilan.motor_front.run_angle(-400,1000)
    # await ilan.wait_for_button()
    # await ilan.motor_front.run_angle(200,1000)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(-10,200)
    # await ilan.wait_for_button()
    # await ilan.turn(30,200)
    # await ilan.wait_for_button()
    # await ilan.drive_straight(-40,200)

async def test():
    pid = {"kp":1, "ki":0.1, "kd": 0.01}
    await ilan.drive_straight(60,200, gradual_stop= False, **pid)


async def main():
    runs = [
        ("2", banana, Icon.EYE_LEFT_BROW),
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
        ("7", whale, Icon.SAD),
        ("T", test),
        ("8", sonar,Icon.HEART),
        ("1", massive, Icon.LEFT),

        # ("9", play_sound)
    ]
    current_run = 0
    print("current", ilan.hub.battery.current(), "voltage", ilan.hub.battery.voltage())
    
            
    while True:
        try:
            if (Button.LEFT in ilan.hub.buttons.pressed()):
                current_run += 1
                if current_run >= len(runs):
                    current_run = 0
                if len(runs[current_run]) ==2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])

            elif (Button.RIGHT in ilan.hub.buttons.pressed()):
                await runs[current_run][1]()

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


