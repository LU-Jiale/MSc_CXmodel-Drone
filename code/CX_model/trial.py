import numpy as np

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
    tn1 = cx.tn1_output(flow)
    tn2 = cx.tn2_output(flow)

    # Update memory for distance just travelled
    memory = cx.cpu4_update(memory, tb1, tn1, tn2)
    cpu4 = cx.cpu4_output(memory)

    # Steer based on memory and direction
    cpu1 = cx.cpu1_output(tb1, cpu4)
    motor = cx.motor_output(cpu1)
    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor

def generate_memory(headings, velocity, cx, bump_shift=0.0, filtered_steps=0.0):
    """For an outbound route, generate all the cell activity."""
    T = len(headings)

    # Initialise TB and memory
    tb1 = np.zeros(central_complex.N_TB1)
    memory = 0.5 * np.ones(central_complex.N_CPU4)

    for t in range(T):
        tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = update_cells(
            heading=headings[t], velocity=velocity[t], tb1=tb1, memory=memory,
            cx=cx, filtered_steps=filtered_steps)

    return tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor


h_out = np.array([0, 0, 0, 0, np.pi/2, np.pi/2,])*(3)
v_out = np.array([[0.0, 0.0], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1], [0.1, 0.1]])*1
cx = cx_basic.CXBasic()
tl2, cl1, tb1, tn1, tn2, memory, cpu4, cpu1, motor = generate_memory(
            headings=h_out, velocity=v_out, cx=cx)
print(motor)



