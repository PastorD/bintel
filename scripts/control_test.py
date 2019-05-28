#!/usr/bin/env python

import control
import numpy as np

import time
import matplotlib.pyplot as plt
import scipy.linalg as la



# x[k+1] = Ax[k] + Bu[k]
A = np.matrix([[0, 1.0], [1, 1]])
B = np.matrix([0.0, 1]).T
Q = np.matrix([[1.0, 0.0], [0.0, 1.0]])
R = np.matrix([[1.0]])
Kopt = None

#K, S, E = control.lqr(A, B, Q, R)

def process(x, u, dt):
    x = x + dt*(A*x + B*u)
    return (x)

def lqr_ref_tracking(x, xref, uref):
    global Kopt
    if Kopt is None:
        #  start = time.time()
        #  Kopt = dlqr_with_iteration(A, B, np.eye(2), np.eye(1))
        Kopt, S, E = control.lqr(A, B, Q, R)

        #  elapsed_time = time.time() - start
        #  print("elapsed_time:{0}".format(elapsed_time) + "[sec]")

    u = -uref - Kopt * (x - xref)

    return u

def main_reference_tracking():
    t = 0.0
    simTime = 10.0
    dt = 0.1

    x = np.matrix([3, 1]).T
    u = np.matrix([0])
    xref = np.matrix([-2, 0]).T
    uref = (A*xref)[1]/B[1]

    time_history = [0.0]
    x1_history = [x[0, 0]]
    x2_history = [x[1, 0]]
    u_history = [1.0]

    while t <= simTime:
        u = lqr_ref_tracking(x, xref, uref)

        u0 = float(u[0, 0])
        x = process(x, u0, dt)

        x1_history.append(x[0, 0])
        x2_history.append(x[1, 0])

        u_history.append(u0)
        time_history.append(t)
        t += dt

    plt.plot(time_history, u_history, "-r", label="input")
    plt.plot(time_history, x1_history, "-b", label="x1")
    plt.plot(time_history, x2_history, "-g", label="x2")
    xref0_h = [xref[0, 0] for i in range(len(time_history))]
    xref1_h = [xref[1, 0] for i in range(len(time_history))]
    plt.plot(time_history, xref0_h, "--b", label="target x1")
    plt.plot(time_history, xref1_h, "--g", label="target x2")

    plt.grid(True)
    plt.xlim([0, simTime])
    plt.title("LQR Tracking")
    plt.legend()
    plt.show()


if __name__ == '__main__':
    print("Start")
    main_reference_tracking()
    print("Done")