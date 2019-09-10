import cvxpy 
import numpy as np
import scipy as sp
import scipy.sparse as sparse
import matplotlib.pyplot as plt
import time
import timeit

# Discrete time model of a quadcopter
Ad = sparse.csc_matrix([
  [1.,      0.,     0., 0., 0., 0., 0.1,     0.,     0.,  0.,     0.,     0.    ],
  [0.,      1.,     0., 0., 0., 0., 0.,      0.1,    0.,  0.,     0.,     0.    ],
  [0.,      0.,     1., 0., 0., 0., 0.,      0.,     0.1, 0.,     0.,     0.    ],
  [0.0488,  0.,     0., 1., 0., 0., 0.0016,  0.,     0.,  0.0992, 0.,     0.    ],
  [0.,     -0.0488, 0., 0., 1., 0., 0.,     -0.0016, 0.,  0.,     0.0992, 0.    ],
  [0.,      0.,     0., 0., 0., 1., 0.,      0.,     0.,  0.,     0.,     0.0992],
  [0.,      0.,     0., 0., 0., 0., 1.,      0.,     0.,  0.,     0.,     0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      1.,     0.,  0.,     0.,     0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      0.,     1.,  0.,     0.,     0.    ],
  [0.9734,  0.,     0., 0., 0., 0., 0.0488,  0.,     0.,  0.9846, 0.,     0.    ],
  [0.,     -0.9734, 0., 0., 0., 0., 0.,     -0.0488, 0.,  0.,     0.9846, 0.    ],
  [0.,      0.,     0., 0., 0., 0., 0.,      0.,     0.,  0.,     0.,     0.9846]
])
Bd = sparse.csc_matrix([
  [0.,      -0.0726,  0.,     0.0726],
  [-0.0726,  0.,      0.0726, 0.    ],
  [-0.0152,  0.0152, -0.0152, 0.0152],
  [-0.,     -0.0006, -0.,     0.0006],
  [0.0006,   0.,     -0.0006, 0.0000],
  [0.0106,   0.0106,  0.0106, 0.0106],
  [0,       -1.4512,  0.,     1.4512],
  [-1.4512,  0.,      1.4512, 0.    ],
  [-0.3049,  0.3049, -0.3049, 0.3049],
  [-0.,     -0.0236,  0.,     0.0236],
  [0.0236,   0.,     -0.0236, 0.    ],
  [0.2107,   0.2107,  0.2107, 0.2107]])
[nx, nu] = Bd.shape

# Constraints
u0 = 10.5916
umin = np.array([9.6, 9.6, 9.6, 9.6]) - u0
umax = np.array([13., 13., 13., 13.]) - u0
xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-0.5,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, 0.5,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

# Objective function
Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
QN = Q
R = 0.1*sparse.eye(4)

# Initial and reference states
x0 = np.zeros(12)
xr = np.array([0.,1.,4.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

# Sizes
ns = 12
nu = 4

# Prediction horizon
N = 30

# Define problem
u = cvxpy.Variable((nu, N))
#x = cvxpy.Variable((nx, N+1))
x_init = cvxpy.Parameter(nx)
objective = 0
constraints = [x[:,0] == x_init]
for k in range(N):
    objective += cvxpy.quad_form(x[:,k] - xr, Q) + cvxpy.quad_form(u[:,k], R)
    constraints += [x[:,k+1] == Ad*x[:,k] + Bd*u[:,k]]
    constraints += [xmin <= x[:,k], x[:,k] <= xmax]
    constraints += [umin <= u[:,k], u[:,k] <= umax]
objective += cvxpy.quad_form(x[:,N] - xr, QN)
prob = cvxpy.Problem(cvxpy.Minimize(objective), constraints)

# Simulate in closed loop
nsim = 100

# Store data Init
xst = np.zeros((ns,nsim))
ust = np.zeros((nu,nsim))

t1 = time.clock()
for i in range(nsim):
    x_init.value = x0
    prob.solve(solver=cvxpy.OSQP, warm_start=True)
    x0 = Ad.dot(x0) + Bd.dot(u[:,0].value)

    # Store Data
    xst[:,i] = x0
    ust[:,i] = u[:,0].value
print(time.clock()-t1)


for i in range(ns):
  plt.plot(range(nsim),xst[i,:],label=str(i))
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
plt.show()    
plt.savefig('cvxpy.png')

savemat('cvxpy.mat', {'xst': xst, 't_eval': t_eval, 'ucvxpy': ust})


for i in range(nu):
  plt.plot(range(nsim),ust[i,:],label=str(i))
plt.plot(range(nsim),np.ones(nsim)*umin[1],label='U_{min}',linestyle='dashed', linewidth=1.5, color='black')
plt.plot(range(nsim),np.ones(nsim)*umax[1],label='U_{max}',linestyle='dashed', linewidth=1.5, color='black')
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
plt.show()  