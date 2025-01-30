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
    ilan.drive_base.drive(151,0)


async def reverse_drive():
    ilan.drive_base.drive(-750, 0)

async def turn_left():
    ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    ilan.drive_base.turn(360, wait=False)

async def front_motor():
    ilan.motor_front.dc(100)

async def back_motor():
    ilan.motor_back.dc(100)

async def front_motor_reverse():
    ilan.motor_front.dc(-50)
    
async def back_motor_reverse():
    ilan.motor_back.dc(-100)

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

@time_it
async def whale():
    pid = {"kp":1, "ki":0, "kd": 0}
    ilan.drive_base.reset()
    await ilan.drive_straight(19)
    # await ilan.wait_for_button(
    # )
    await ilan.turn(-15)
    # await ilan.wait_for_button()
    await multitask(ilan.drive_straight(50,200, **pid), prepare_whale_motor())
    await ilan.wait_for_button(debug=False)
    await ilan.turn(55)
    # await ilan.wait_for_button()
    await ilan.drive_straight(20,150, **pid)
    await wait(1000)
    await ilan.drive_straight(-2,150, **pid)
    # await ilan.wait_for_button()
    await ilan.drive_straight(4, **pid)
    # await ilan.wait_for_button()
    await multitask(ilan.drive_straight(-28,100, **pid), ilan.motor_back.run_angle(250,-290))
    await ilan.turn(120)
    # await ilan.wait_for_button()
    await ilan.motor_back.run_angle(250,90)
    await ilan.motor_back.run_angle(125,120)
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
    await ilan.drive_straight(-30,300)
    await ilan.turn(90)
    await ilan.drive_straight(13)
    await ilan.run_back_motor(50,-120)
    await ilan.turn(-5)
    await ilan.drive_straight(-53)
    await ilan.run_back_motor(300,-90)

@time_it
async def banana():
    pid = {"kp":0.9, "ki": 0, "kd": 0}
    # await ilan.motor_back.run_angle(100,25)
    await ilan.drive_straight(44,200, gradual_stop= False, **pid)
    await ilan.drive_straight(-13,300, **pid)
    await ilan.run_front_motor(310,300)
    await ilan.turn(40)
    await ilan.drive_straight(37,230,**pid)
    await ilan.turn(35)
    await ilan.drive_straight(17,260,**pid)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.drive_straight(-20, 450)
    await ilan.turn(-60)
    # await ilan.wait_for_button()
    await ilan.drive_straight(-50, 500, gradual_start=False, gradual_stop=False)
    # await ilan.wait_for_button()
    # await ilan.turn(-67)
    # await ilan.drive_straight(-10,260,**pid)
    # # await ilan.wait_for_button()
    # await ilan.wait_for_button()
    # await ilan.run_back_motor(205,750)
    # await ilan.drive_straight(-11,500, gradual_start=False, gradual_stop=False)

@time_it
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

    
@time_it
async def green():
    pid = {"kp": 1, "ki": 0, "kd": 0}
    Debug = False
    await ilan.turn(23,100)
    # await ilan.wait_for_button(Debug)
    await ilan.drive_straight(48,200, **pid)
    await ilan.turn(65,150)
    await ilan.wait_for_button(Debug)
    await wait(100)
    await ilan.drive_straight(20,200, **pid)
    await ilan.drive_straight(-14,200, **pid)
    await ilan.turn(135, 150)
    await ilan.drive_straight(-27,100,**pid)
    await ilan.wait_for_button(Debug)
    await ilan.turn(-40,100)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(-18,200,gradual_stop=True, **pid)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(1,300)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(-700,1500)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(500,1500)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(7,700,gradual_stop=False ,**pid)
    await ilan.turn(35)
    await ilan.drive_straight(60,700,gradual_stop=False)

@time_it
async def massive():
    debug= False
    pid = {"kp": 1, "ki": 0, "kd": 0.0}
    await ilan.drive_straight(52,300, **pid)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(150,175)
    await ilan.wait_for_button(debug)
    # await ilan.turn(-4,200)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(15,300, **pid)
    await ilan.wait_for_button(debug)
    # await ilan.drive_straight(-1.5,200)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(200,-55)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,-400)
    await wait(500) 
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,10)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(400,650)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-10,200, **pid)
    await ilan.wait_for_button(debug)
    await ilan.turn(20)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(-85,500, **pid)
    await ilan.wait_for_button(debug= False)

@time_it
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
        ("2", green, Icon.FALSE)

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
                print("run ended!!!")

            elif (Button.BLUETOOTH in ilan.hub.buttons.pressed()):
                await banana()
                print("test ended!!!")
            else:
                await stop_all()
        except Exception as e:
            print(e)
            raise e
        finally:
            await wait(100)


run_task(main())




