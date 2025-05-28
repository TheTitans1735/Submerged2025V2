from pybricks.pupdevices import ColorSensor
from pybricks.parameters import Port
from pybricks.tools import wait

# Initialize the sensor.
sensor = ColorSensor(Port.A)

while True:
    # Read the color and reflection
    color = sensor.color()
    reflection = sensor.reflection()

    # Print the measured color and reflection.
    print(color, reflection)

    # Move the sensor around and see how
    # well you can detect colors.

    # Wait so we can read the value.
    wait(100)
"""
controller = XboxController()

# פונקציה שרצה כאשר הג'ויסטיק הימני מוסט
def do_something():
    print("הג'ויסטיק הימני הופעל!")

while True:
    # קבלת ערכים מהג'ויסטיק הימני (ערכים בין -100 ל-100)
    rx = controller.joystick_right[0]
    ry = controller.joystick_right[1]

    print(f"Right Joystick - X: {rx}, Y: {ry}")

    # אם ערך X של הג'ויסטיק הימני גבוה מ־60, הפעל פונקציה
    if rx > 60:
        do_something()

    wait(100)"""