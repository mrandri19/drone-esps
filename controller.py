from serial import Serial
from time import sleep, time
from xbox360controller import Xbox360Controller

# Writing to the console at 50Hz seems to be the maximum frequency that the
# system can handle without joining packets toghether. How to benchmark this
# well? Raising the baudrate could work I suppose, as well as sending less data
# by packing it more efficiently maybe? Also how to measure latency?
FREQUENCY = 50
PERIOD = 1 / FREQUENCY

BAUDRATE = 921600
PORT = "/dev/ttyUSB0"


fsm_state = "START"
data = None


def on_button_a_pressed(button):
    global fsm_state, data
    fsm_state = "DATA_AVAILABLE"
    data = "Button a was pressed"
    print(data)


def o_axis_r_moved(axis):
    global fsm_state, data
    fsm_state = "DATA_AVAILABLE"
    data = f"Axis {axis.name} moved to {axis.x} {axis.y}"
    print(data)


i = 0
with Xbox360Controller(axis_threshold=0.05) as c:
    c.button_a.when_pressed = on_button_a_pressed
    # c.axis_r.when_moved = o_axis_r_moved

    with Serial(port=PORT, baudrate=BAUDRATE) as s:
        fsm_state = "SERIAL_OPEN"
        fsm_state = "NO_DATA_TO_SEND"

        while True:
            if fsm_state == "DATA_AVAILABLE":
                tic = time()
                s.write(f"-{i}-".encode())

                fsm_state = "NO_DATA_TO_SEND"
                data = None
                i += 1

                toc = time()
                sleep(PERIOD - (toc - tic))
