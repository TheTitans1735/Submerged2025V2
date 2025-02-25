from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.robotics import DriveBase
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch, run_task, multitask
from pybricks.parameters import Icon, Color, Button, Direction
from robot import Robot,time_it
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
    ilan.drive_base.drive(100,0)


async def reverse_drive():
    ilan.drive_base.drive(-750, 0)

async def turn_left():
    ilan.drive_base.turn(-360, wait=False)

async def turn_right():
    ilan.drive_base.turn(360, wait=False)

async def front_motor():
    ilan.motor_front.dc(50)

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
    # await ilan.run_back_motor(100,100)
    # await wait(1000)
    await ilan.run_back_motor(200,-290)


@time_it
async def whale():
    """
    ביצוע משימת הלוויתן.
    """
    debug= True
    pid = {"kp":1, "ki":0, "kd": 0}
    ilan.drive_base.reset()
    # await ilan.hub.speaker.beep(duration=700)

    await multitask(ilan.drive_straight(28,850),prepare_whale_motor())
    await ilan.wait_for_button(debug = False)    
    await ilan.turn(-28)
    await ilan.wait_for_button(debug = False)
    await ilan.drive_straight(40,500, **pid)
    await ilan.wait_for_button(debug = False)
    await ilan.turn(74)
    await ilan.wait_for_button(debug = False)
    await ilan.drive_straight(32,650,gradual_stop=False, **pid)    # await wait(1000)
    # await ilan.drive_straight(-2,700)
    # await ilan.wait_for_button()
    # await ilan.turn(3)
    # await ilan.drive_straight(4,200,gradual_stop=False, **pid)
    # await ilan.wait_for_button()
    await multitask(ilan.drive_straight(-32,700, **pid), ilan.run_back_motor(250,-290))
    await ilan.turn(125)
    # await ilan.wait_for_button()
    await ilan.run_back_motor(250,280)
    # await ilan.wait_for_button()
    # await ilan.motor_back.run_angle(150  ,140 )
    await ilan.drive_straight(-25,900,**pid,)
    # await ilan.turn(14)
    # await ilan.drive_straight(8)
    # await multitask(ilan.drive_straight(-9, 900,**pid))
    await ilan.run_back_motor(333,-25)        
    await ilan.drive_straight(19)
    # await ilan.wait_for_button(
    # )
    await ilan.turn(-15)
    # await ilan.wait_for_button()
    # await multitask(ilan.drive_straight(55,700,gradual_stop=False ,**pid), ilan.run_back_motor(800,-290))
    # await ilan.run_back_motor(200,290)

async def sonar():
    await ilan.drive_straight(-30,300)
    await ilan.turn(80,200)
    await ilan.drive_straight(-13)
    await ilan.run_back_motor(150,-210)
    await ilan.drive_straight(-53,10)
    await ilan.run_back_motor(300,-90)
async def pick_up():
    pid = { "kp": 3, "ki": 0.1, "kd": 0.01}
    await ilan.drive_straight(46 ,150,gradual_stop=False, **pid)
    await ilan.drive_straight(-14, **pid)
    await ilan.run_front_motor(310,300)
    await ilan.turn(42 )
    await ilan.drive_straight(41, **pid)
    await ilan.turn(35)
    await ilan.drive_straight(12, **pid)
    await ilan.run_front_motor(200,-300)
    await ilan.drive_straight(-20,450, **pid)
    await ilan.turn(-65)
    await ilan.drive_straight(-65,700, **pid,gradual_stop=False)
@time_it
async def banana():
    pid = {"kp":0.9, "ki": 0, "kd": 0}
    # await ilan.motor_back.run_angle(100,25)
    await ilan.drive_straight(47,200, gradual_stop= False, **pid)
    await ilan.drive_straight(-14,300, **pid)
    await ilan.run_front_motor(310,300)
    await ilan.turn(40)
    await ilan.drive_straight(42,230,**pid)
    await ilan.turn(35)
    await ilan.drive_straight(15,260,**pid)
    await ilan.motor_front.run_angle(200,-300)
    await ilan.drive_straight(-20, 450)
    await ilan.turn(-80)
    # await ilan.wait_for_button()
    await ilan.drive_straight(-65, 500, gradual_start=False, gradual_stop=False)
    # await ilan.wait_for_button()
    # await ilan.turn(-67)
    # await ilan.drive_straight(-10,260,**pid)
    # # await ilan.wait_for_button()

@time_it
async def crabs():
    # await ilan.run_back_motor(-200,200)
    await ilan.drive_straight(-107,800, gradual_start=False, kp = 0, ki = 0, kd = 0)
    await ilan.motor_back.run_time(200,2500)
    await ilan.drive_straight(12,120, kp = 0, ki = 0, kd = 0)
    await multitask(ilan.drive_straight(25,450, kp = 0, ki = 0, kd = 0), ilan.run_back_motor(200, -100))
    await ilan.turn(-31,100)
    await ilan.drive_straight(-35,450, kp = 0, ki = 0, kd = 0)
    await ilan.turn(29,50)
    await ilan.drive_straight(-50,450, kp = 0, ki = 0, kd = 0)
    await ilan.turn(20,150)
    await ilan.drive_straight(-40,900, kp = 0, ki = 0, kd = 0)


@time_it
async def green():
    # pid = {"kp": 1, "ki": 0, "kd": 0}
    Debug = False
    await ilan.turn(23,100)
    # await ilan.wait_for_button(Debug)
    await ilan.drive_straight_1(48, 700)
    await ilan.turn(65,150)
    await ilan.wait_for_button(Debug)
    await wait(100)
    await ilan.drive_straight_1(20, 700)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight_1(-14, 700)
    await ilan.turn(130, 150)
    await ilan.drive_straight_1(-27, 700)
    await ilan.wait_for_button(Debug)
    await ilan.turn(-40,100)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight_1(-18, 700,gradual_stop=True)
    await ilan.wait_for_button(Debug)
    await ilan.drive_straight(3, 700)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(-700,1500)
    await ilan.wait_for_button(Debug)
    await ilan.motor_back.run_time(500,1500)
    # await ilan.wait_for_button(Debug)
    # await ilan.drive_straight(7,500,gradual_stop=False ,**pid)
    await ilan.drive_straight(7,700)
    await ilan.turn(35)
    await ilan.drive_straight(75,700)

@time_it
async def coral():
    await ilan.drive_straight(8,100)
    await ilan.drive_straight(-8,gradual_start=False,gradual_stop=False)

@time_it
async def massive():
    debug= False
    pid = {"kp": 1, "ki": 0, "kd": 0.0}
    await ilan.drive_straight(52,300, **pid)
    await ilan.wait_for_button(debug)
    await ilan.run_back_motor(100,175)
    await ilan.wait_for_button(debug)
    # await ilan.turn(-4,200)
    await ilan.wait_for_button(debug)
    await ilan.drive_straight(14,300, **pid)
    await ilan.wait_for_button(debug)
    # await ilan.drive_straight(-1.5,200)
    await ilan.wait_for_button(debug)
    await ilan.motor_back.run_time(-200,1000)
    await ilan.wait_for_button()
    await ilan.motor_front.run_angle(200,-400)
    await wait(500) 
    await ilan.wait_for_button(debug)
    #await ilan.turn(10,200)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(200,5)
    await ilan.wait_for_button(debug)
    await ilan.motor_front.run_angle(400,500)
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


async def test():
    # await ilan.hub.speaker.beep()
    voltage = ilan.hub.battery.voltage()
    await ilan.hub.display.text(str(voltage))
    if voltage > 7500:
        await ilan.hub.display.icon(Icon.HAPPY)
        await wait(1000)
    else:
        await ilan.hub.display.icon(Icon.SAD)
        await wait(2000)
    # await ilan.hub.display.animate(colection[Icon.house, Icon.SAD],interval=7000)

async def test9():
   pid = {"kp": 1, "ki": 0, "kd": 0}
   await ilan.drive_straight(52,300, **pid)

async def test8():
   pid = {"kp": 1, "ki": 0, "kd": 0}
   await ilan.drive_straight(52,300, **pid)

async def battery_check():
    pass


# this is the main program
async def main():
    runs = [
        ("0", battery_check, Icon.TRIANGLE_UP),
        ("1", massive, Icon.LEFT),
        ("2", green, Icon.FALSE), 
        ("3", crabs, Icon.HAPPY),
        ("4", pick_up, Icon.SAD),
        ("5", coral, Icon.PAUSE),
        ("7", whale, Icon.FULL),
        (" ", drive, Icon.ARROW_LEFT),
        (" ", reverse_drive, Icon.ARROW_RIGHT),
        (" ", turn_left, Icon.ARROW_LEFT_DOWN),
        (" ", turn_right, Icon.ARROW_LEFT_UP),
        ("1", front_motor),
        ("2", back_motor),
        ("3", front_motor_reverse),
        ("4", back_motor_reverse),
        (" ", nigg, Icon.CIRCLE),
        (" ", turn, Icon.CLOCKWISE),
        (" ", sonar,Icon.HEART),
        ("T", test), 
    ]
    current_run = 0
    await ilan.buttery_status()     

    buttery_status_timer = StopWatch()   
    while True:
        if buttery_status_timer.time()> 10000:
            await ilan.buttery_status()
            buttery_status_timer.reset()
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
                current_run -= 1
                if current_run < 0:
                    current_run = len(runs)-1
                if len(runs[current_run]) ==2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])

            elif (Button.BLUETOOTH in ilan.hub.buttons.pressed()):
                await runs[current_run][1]()

                current_run += 1
                if current_run >= len(runs):
                    current_run = 0
                if len(runs[current_run]) ==2:
                    ilan.hub.display.char(runs[current_run][0])
                else:
                    ilan.hub.display.icon(runs[current_run][2])
            else:
                await stop_all()
        except Exception as e:
            print(e)
            raise e
        finally:
            await wait(100)


run_task(main())