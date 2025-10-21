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

    # cmd = ["turn cw90", "turn ccw90", "forward", "backward"]
    frame2_data = {"turn cw":[0x7f, 0x60, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "turn ccw":[0x7f, 0x9e, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "forward":[0x7f, 0x7f, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "forward cw": [0x7f, 0x22, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "forward ccw": [0x7f, 0xa2, 0x44, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "backward":[0x7f, 0x7f, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "backward cw": [0x7f, 0x22, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
                   "backward ccw": [0x7f, 0xaa, 0xbb, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f],
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

        # cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task(ch_a, start, "forward", sec))
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

        # cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task(ch_a, start, "forward ccw", sec))
        print(thread)
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)


# def turnCW(angle):
#     with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
#         for ch in [ch_a]:
#             ch.busOn()
#
#         start = time.time()
#         print(start)
#
#         # cmd_list = ["turn cw", "turn ccw", "forward", "backward"]
#         sec = calc_angle2time(angle)
#
#         thread = threading.Thread(target=long_running_task(ch_a, start, "turn cw", 8.7))
#         thread.start()
#         thread.join()
#
#         frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
#         ch_a.write(frame2)
#
#         stop = time.time()
#         print(stop - start)

def turnCW(angle):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        # cmd_list = ["turn cw", "turn ccw", "forward", "backward"]
        sec = calc_angle2time(angle)

        thread = threading.Thread(target=long_running_task(ch_a, start, "turn cw", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)


# def turnCCW(angle):
#     with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
#         for ch in [ch_a]:
#             ch.busOn()
#
#         start = time.time()
#         print(start)
#
#         # cmd_list = ["turn cw", "turn ccw", "forward", "backward"]
#         sec = calc_angle2time(angle)
#
#         thread = threading.Thread(target=long_running_task(ch_a, start, "turn ccw", 8.7))
#         thread.start()
#         thread.join()
#
#         frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
#         ch_a.write(frame2)
#
#         stop = time.time()
#         print(stop - start)

def turnCCW(angle):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        # cmd_list = ["turn cw", "turn ccw", "forward", "backward"]
        sec = calc_angle2time(angle)

        thread = threading.Thread(target=long_running_task(ch_a, start, "turn ccw", sec))
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

        # cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task(ch_a, start, "backward", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)

def go_backward_cw(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        # cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task(ch_a, start, "backward cw", sec))
        thread.start()
        thread.join()

        frame2 = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(frame2)

        stop = time.time()
        print(stop - start)

def go_backward_ccw(cm):
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        for ch in [ch_a]:
            ch.busOn()

        start = time.time()
        print(start)

        # cmd_list = ["turn cw90", "turn ccw90", "forward", "backward"]
        sec = calc_distance2time(cm)

        thread = threading.Thread(target=long_running_task(ch_a, start, "backward ccw", sec))
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




#--------------------------------------------------------------------------------------------------


def fork_up(cm):
    """지정된 거리(cm)만큼 포크를 올립니다."""
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        # TODO: 'calc_function.py'에 아래 함수를 실제 포크 상승 속도에 맞게 추가해야 합니다.
        # 예시: def calc_fork_height2time(cm): return cm / 5.0  # 1초에 5cm 상승
        # sec = calc_fork_height2time(cm)

        sec = calc_distance2time(cm)

        # TODO: 'long_running_task'에 "fork_up" case와 CAN 데이터를 추가해야 합니다.
        thread = threading.Thread(target=long_running_task(ch_a, start, "fork_up", sec))
        thread.start()
        thread.join()

        # STOP Frame 전송
        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- 포크를 {cm}cm 만큼 올렸습니다. ({sec:.2f}초 실행) ---")


def fork_down(cm):
    """지정된 거리(cm)만큼 포크를 내립니다."""
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        # TODO: 'calc_function.py'에 calc_fork_height2time 함수를 추가해야 합니다.
        # sec = calc_fork_height2time(cm)

        sec = calc_distance2time(cm)

        # TODO: 'long_running_task'에 "fork_down" case와 CAN 데이터를 추가해야 합니다.
        thread = threading.Thread(target=long_running_task(ch_a, start, "fork_down", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- 포크를 {cm}cm 만큼 내렸습니다. ({sec:.2f}초 실행) ---")


def fork_forward(cm):
    """지정된 거리(cm)만큼 포크를 전진시킵니다."""
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        # TODO: 'calc_function.py'에 아래 함수를 실제 포크 전/후진 속도에 맞게 추가해야 합니다.
        # 예시: def calc_fork_dist2time(cm): return cm / 10.0 # 1초에 10cm 이동
        # sec = calc_fork_dist2time(cm)

        sec = calc_distance2time(cm)

        # TODO: 'long_running_task'에 "fork_forward" case와 CAN 데이터를 추가해야 합니다.
        thread = threading.Thread(target=long_running_task(ch_a, start, "fork_forward", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- 포크를 {cm}cm 만큼 전진시켰습니다. ({sec:.2f}초 실행) ---")


def fork_backward(cm):
    """지정된 거리(cm)만큼 포크를 후진시킵니다."""
    with canlib.openChannel(0, bitrate=canlib.Bitrate.BITRATE_500K) as ch_a:
        ch_a.busOn()
        start = time.time()

        # TODO: 'calc_function.py'에 calc_fork_dist2time 함수를 추가해야 합니다.
        # sec = calc_fork_dist2time(cm)

        sec = calc_distance2time(cm)

        # TODO: 'long_running_task'에 "fork_backward" case와 CAN 데이터를 추가해야 합니다.
        thread = threading.Thread(target=long_running_task(ch_a, start, "fork_backward", sec))
        thread.start()
        thread.join()

        stop_frame = Frame(id_=0x01E3, data=[0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f])
        ch_a.write(stop_frame)
        print(f"--- 포크를 {cm}cm 만큼 후진시켰습니다. ({sec:.2f}초 실행) ---")


# 처음 can에 연결하는 경우 시간을 좀 줘야함.