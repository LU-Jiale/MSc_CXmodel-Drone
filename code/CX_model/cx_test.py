import os
import numpy as np
import time
import central_complex
import cx_rate
import cx_basic


def update_cells(heading, velocity, tb1, memory, cx, filtered_steps=0.0):
    """Generate activity for all cells, based on previous activity and current
    motion."""
    # Compass
    tl2 = cx.tl2_output(heading)
    cl1 = cx.cl1_output(tl2)
    tb1 = cx.tb1_output(cl1, tb1)

    # Speed
    flow = cx.get_flow(heading, velocity, filtered_steps)
    tn1 = cx.tn1_output(velocity)
    tn2 = cx.tn2_output(velocity)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

# initialize CX model
#cx = cx_basic.CXBasic()
cx = cx_rate.CXRate(noise=0)
tb1 = np.zeros(central_complex.N_TB1)
memory = 0.5 * np.ones(central_complex.N_CPU4)

T = 200
velocity = np.ones([T,2], dtype=float)
headings = np.zeros(T, dtype=float)
headings[(T/3):-1] = -np.pi/2

for t in range(T):
        tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=headings[t], velocity=velocity[t], tb1=tb1, memory=memory, cx=cx)

        angle, distance = cx.decode_cpu4(cpu4)
        angle_degree = angle/np.pi * 180
        print "Angle:%.2f  Distance:%.2f Motor:%.2f" % (angle_degree, distance, motor)
        time.sleep(0.1)
