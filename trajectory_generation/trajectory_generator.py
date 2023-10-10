import numpy as np
import matplotlib.pyplot as plt
import scienceplots


class Trajectory:
    """
    Trajectory class
    """
    def __init__(self, t,  q, qd, qdd):
        self.t = t
        self.q = q
        self.qd = qd
        self.qdd = qdd

    def plot(self):
        with plt.style.context(['science', 'no-latex', 'high-vis']):
            ax = plt.subplot(3, 1, 1)
            ax.grid(True)
            ax.plot(self.t, self.q, 'r')
            print("q: ", self.q)
            ax.set_ylabel("${{q}}(t)$")
            ax = plt.subplot(3, 1, 2)
            ax.grid(True)
            ax.plot(self.t, self.qd, 'g')
            ax.set_ylabel("$\dot{{q}}(t)$")
            ax = plt.subplot(3, 1, 3)
            ax.grid(True)
            ax.plot(self.t, self.qdd, 'b')
            ax.set_ylabel(f"$\ddot{{q}}(t)$")
            ax.set_xlabel("t (seconds)")
            plt.show()

def quintic(q0, qf, t, qd0=0, qdf=0):
    """
    quintic funtion 
    """
    if isinstance(t, int):
        t = np.arange(0, t)
        print("t: ", t)
    tf = max(t)
    polyfunc = quintic_func(q0, qf, tf, qd0, qdf)
    trajec = polyfunc(t)
    p   = trajec[0]
    pd  = trajec[1]
    pdd = trajec[2]
    return Trajectory(t, p , pd, pdd)

def quintic_func(q0, qf, T, qd0 =0 , qdf =0):
    """
    quintic sub-funtion 
    """
    A = [
        [ 0.0,          0.0,         0.0,        0.0,     0.0,  1.0],
        [ T**5,         T**4,        T**3,       T**2,    T,    1.0],
        [ 0.0,          0.0,         0.0,        0.0,     1.0,  0.0],
        [ 5.0 * T**4,   4.0 * T**3,  3.0 * T**2, 2.0 * T, 1.0,  0.0],
        [ 0.0,          0.0,         0.0,        2.0,     0.0,  0.0],
        [20.0 * T**3,  12.0 * T**2,  6.0 * T,    2.0,     0.0,  0.0],
    ]
    b = np.r_[q0, qf, qd0, qdf, 0, 0]
    # calculate coefficients by using  
    # least-squares solution to a linear matrix equation
    # A @ x = b -> find x 
    coeffs, resid, rank, s = np.linalg.lstsq(A, b , rcond=None)

    # coefficients of derivatives
    coeffs_d = coeffs[0:5] * np.arange(5, 0, -1)
    coeffs_dd = coeffs_d[0:4] * np.arange(4, 0, -1)

    return lambda t: (
        np.polyval(coeffs, t),
        np.polyval(coeffs_d, t),
        np.polyval(coeffs_dd, t),
    )


def cubic(q0, qf, t, qd0=0, qdf=0):
    """
    cubic funtion 
    """
    if isinstance(t, int):
        t = np.arange(0, t)
        print("t: ", t)
    tf = max(t)
    polyfunc = cubic_func(q0, qf, tf, qd0, qdf)
    trajec = polyfunc(t)
    p   = trajec[0]
    pd  = trajec[1]
    pdd = trajec[2]
    return Trajectory(t, p , pd, pdd)

def cubic_func(q0, qf, T, qd0=0, qdf =0):
    """
    cubic function
    """
    A = [
        [ 0.0,      0.0,        0.0,      1.0],
        [ T**3,     T**2,       T,        1.0], 
        [ 0.0,      0.0,        1,        0.0],
        [3*T**2,    2*T,        1.0,      0.0],
    ]
    b = np.r_[q0, qf, qd0, qdf]
    # calculate coefficients by using  
    # least-squares solution to a linear matrix equation
    # A @ x = b -> find x 
    coeffs, resid, rank, s = np.linalg.lstsq(A, b , rcond=None)

    # coefficients of derivatives
    coeffs_d = coeffs[0:3] * np.arange(3, 0, -1)
    coeffs_dd = coeffs_d[0:2] * np.arange(2, 0, -1)

    return lambda t: (
        np.polyval(coeffs, t),
        np.polyval(coeffs_d, t),
        np.polyval(coeffs_dd, t),
    )


quintic_1 = quintic(1, 2, 100)
quintic_1.plot()

# cubic_1 = cubic(1, 2, 100)
# cubic_1.plot()