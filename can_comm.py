from canlib import canlib, Frame
import time
import keyboard
import threading
import asyncio


async def async_time_buffer(seconds):
    await asyncio.sleep(seconds)


def long_running_task(ch_a, start, cmd, sec):
    print("Long running task started.")

    # cmd = ["turn cw90", "turn ccw90", "forward", "backward"]
    frame2_data = {"turn cw90":[0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "turn ccw90":[0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "forward":[0x7f, 0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "backward":[0x7f, 0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f]}

    while not keyboard.is_pressed('q'):
        frame1 = Frame(id_=0x02E4, data=[0x42, 0x00, 0x00, 0x0A, 0x0A, 0x40, 0x69, 0x93])
        frame2 = Frame(id_=0x01E4, data=frame2_data[cmd])

        ch_a.write(frame1)
        ch_a.write(frame2)
        asyncio.run(async_time_buffer(0.06))
        stop = time.time()
        print(stop - start)
        if (stop - start) > sec:
            break
        pass
    print("Long running task stopped.")


with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:

    for ch in [ch_a]:
        ch.busOn()

    start = time.time()
    print(start)

    cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
    cmd = cmd_list[1]
    sec = 2

    thread = threading.Thread(target=long_running_task(ch_a, start, cmd, sec))
    thread.start()
    thread.join()

    frame2 = Frame(id_=0x01E4, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
    ch_a.write(frame2)

    stop = time.time()
    print(stop-start)
