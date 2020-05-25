from serial import Serial
from time import sleep, time

# Writing to the console at 50Hz seems to be the maximum frequency that the
# system can handle without joining packets toghether. How to benchmark this
# well? Raising the baudrate could work I suppose, as well as sending less data
# by packing it more efficiently maybe? Also how to measure latency?
FREQUENCY = 1000
PERIOD = 1 / FREQUENCY

BAUDRATE = 921600
PORT = "/dev/ttyUSB0"

i = 0
with Serial(port=PORT, baudrate=BAUDRATE) as s:
    while True:
        tic = time()
        s.write(f"-{i}-".encode())
        i += 1
        toc = time()
        sleep(PERIOD - (toc - tic))
