from canlib import canlib, Frame
import time
import keyboard
import threading
import asyncio
from calc_function import *


async def async_time_buffer(seconds):
    await asyncio.sleep(seconds)


def long_running_task(ch_a, start, cmd, sec):
    print("Long running task started.")

    frame2_data = {"turn cw":[0x7f, 0x60, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "turn ccw":[0x7f, 0x9e, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "forward":[0x7f, 0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "backward":[0x7f, 0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "stop" : [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "fork_up": [0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "fork_down":[0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "fork_forward": [0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x44, 0x7f, 0x7f],
                   "fork_backward":[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0xbb, 0x7f, 0x7f]}

    while not keyboard.is_pressed('q'):
        frame1 = Frame(id_=0x02E3, data=[0x42, 0x00, 0x00, 0x0A, 0x0A, 0x40, 0x69, 0x93])
        frame2 = Frame(id_=0x01E3, data=frame2_data[cmd])

        ch_a.write(frame1)
        ch_a.write(frame2)
        asyncio.run(async_time_buffer(0.06))
        stop = time.time()
        print(f"{stop - start:.2f}")
        if (stop - start) > sec:
            break
        pass
    print("Long running task stopped.")


def go_forward(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "forward", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)


def go_forward_ccw(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "forward ccw", sec))
        print(thread)
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)

def turnCW(angle):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        sec = calc_angle2time(angle)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "turn cw", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)

def turnCCW(angle):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        sec = calc_angle2time(angle)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "turn ccw", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)


def go_backward(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "backward", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)

def move_right(p_distance):
    turnCW(30)
    time.sleep(3)
    go_forward(p_distance)
    time.sleep(3)
    turnCCW(30)
    time.sleep(3)


def move_left(p_distance):
    turnCCW(30)
    time.sleep(3)
    go_forward(p_distance)
    time.sleep(3)
    turnCW(30)
    time.sleep(3)


def fork_up(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "fork_up", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- Fork lifted by {cm}cm. (Ran for {sec:.2f} seconds) ---")


def fork_down(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "fork_down", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- Fork lowered by {cm}cm. (Ran for {sec:.2f} seconds) ---")


def fork_forward(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "fork_forward", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- Fork moved forward by {cm}cm. (Ran for {sec:.2f} seconds) ---")


def fork_backward(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task, args=(ch_a, start, "fork_backward", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- Fork moved backward by {cm}cm. (Ran for {sec:.2f} seconds) ---")
