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
    ilan.drive_base.drive(100,0)


async def reverse_drive():
    ilan.drive_base.drive(-750, 0)

async def turn_left():
    await ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    await ilan.drive_base.turn(360, wait=False)

async def front_motor():
    ilan.motor_front.dc(300)

async def back_motor():
    ilan.motor_back.dc(500)

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
    await multitask(ilan.drive_straight(65,200, **pid), prepare_whale_motor())
    await ilan.wait_for_button(debug=False)
    await ilan.turn(78)
    await ilan.wait_for_button(debug=True)
    await ilan.drive_straight(30,150, **pid)
    await wait(1000)
    await ilan.drive_straight(-2,150, **pid)
    await ilan.wait_for_button()
    await ilan.drive_straight(7, **pid)
    await multitask(ilan.drive_straight(-29,150, **pid), ilan.motor_back.run_angle(250,-290))
    await ilan.turn(106)
    await ilan.motor_back.run_angle(250,90)
    await ilan.motor_back.run_angle(80,120)
    await ilan.drive_straight(-100,200,**pid)
    # await ilan.turn(14)
    # await ilan.drive_straight(8)
    await multitask(ilan.drive_straight(-9, **pid))
    await ilan.run_back_motor(200, -120)

async def sonar():
    await ilan.drive_straight(-30,300)
    await ilan.turn(80,200)
    await ilan.drive_straight(-13)
    await ilan.run_back_motor(150,-210)
    await ilan.drive_straight(-53,10)
    await ilan.run_back_motor(300,-90)

async def banana():
    await ilan.drive_straight(43,320)
    await ilan.drive_straight(-20,320)
    await ilan.run_front_motor(310,300)
    await ilan.turn(43,170)
    await ilan.drive_straight(33,230)
    await ilan.turn(36,170)
    await ilan.drive_straight(8,260)
    await ilan.turn(28,170)
    await ilan.drive_straight(6,200)
    await ilan.run_front_motor(320,-300)
    await ilan.drive_straight(-10,260)
    await ilan.turn(-133,170)
    await ilan.drive_straight(-7,330)
    await ilan.run_back_motor(200,360)
    await ilan.drive_straight(-5,370)

async def crabs():
    # await ilan.run_back_motor(-200,200)
    await ilan.drive_straight(-107,800, gradual_start=False, kp = 0, ki = 0, kd = 0)
    await ilan.run_back_motor(200,350)
    await ilan.drive_straight(11,150, kp = 0, ki = 0, kd = 0)
    await multitask(ilan.drive_straight(25,450, kp = 0, ki = 0, kd = 0), ilan.run_back_motor(200, -100))
    await ilan.turn(-31,100)
    await ilan.drive_straight(-35,450, kp = 0, ki = 0, kd = 0)
    await ilan.turn(29,50)
    await ilan.drive_straight(-50,450, kp = 0, ki = 0, kd = 0)
    await ilan.turn(20,150)
    await ilan.drive_straight(-40,900, kp = 0, ki = 0, kd = 0)


async def massive():
    pid = {"kp": 1, "ki": 0, "kd": 0}
    await ilan.drive_straight(52,300, **pid)
    await ilan.run_back_motor(200,160)
    await ilan.turn(-4,200)
    await ilan.drive_straight(18.5,300, gradual_start=False, **pid)
    await ilan.wait_for_button()
    await ilan.drive_straight(-2.5,300)
    await ilan.wait_for_button()
    await ilan.run_back_motor(200,-50)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(200,-400)
    await ilan.wait_for_button()
    await ilan.turn(10,200)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(200,5)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(400,650)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(200,-300)
    await ilan.wait_for_button()
    await ilan.drive_straight(-10,200, **pid)
    await ilan.wait_for_button()
    await ilan.turn(30)
    await ilan.wait_for_button()
    await ilan.drive_straight(-40,500, **pid)


async def test():
   pid = {"kp": 1, "ki": 0, "kd": 0}
   await ilan.drive_straight(52,300, **pid)



# this is the main program
async def main():
    runs = [
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
        ("7",whale, Icon.FULL),
        ("T", test),
        ("8", sonar,Icon.HEART),
        ("1", massive, Icon.LEFT),
        ("2", banana, Icon.SAD)
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