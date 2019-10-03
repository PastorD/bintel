#%% Initial Data

import osqp
import numpy as np
import scipy as sp
import scipy.sparse as sparse
import matplotlib.pyplot as plt
import time
import matplotlib.cm as cm
import matplotlib as mpl

mpl.rcParams['mathtext.fontset'] = 'cm'
mpl.rcParams['mathtext.rm'] = 'serif'

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

# Sizes
ns = 12
nu = 4

# Constraints
u0 = 10.5916 # Hover Thrust
umin = np.ones(nu)*9.6 - u0
umax = np.ones(nu)*11.0 - u0
xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])


umin = np.ones(nu)*-100 - u0
umax = np.ones(nu)*100.0 - u0
xmin = 100*np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
xmax = 100*np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])


# Objective function
# Objective function
Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
QN = Q
R = 0.1*sparse.eye(nu)

#%% 

# Initial and reference states
x0 = np.array([0.,0.,20.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
xr = np.zeros(ns)#np.linspace(0,10,ns) #np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])

# Prediction horizon
N = 10

# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)]).tocsc()
# - linear objective
q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
               np.zeros(N*nu)])
# - linear dynamics
Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(N*nx)])
ueq = leq
# - input and state constraints
Aineq = sparse.eye((N+1)*nx + N*nu)
lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
# - OSQP constraints
A = sparse.vstack([Aeq, Aineq]).tocsc()
l = np.hstack([leq, lineq])
u = np.hstack([ueq, uineq])

#! Visualize Matrices
fig = plt.figure()
fig.suptitle("QP Matrices to solve MP in sparse form. N={}, ns={}, nu={}".format(N,ns,nu),fontsize=20)
plt.subplot(2,4,1,xlabel="Ns*(N+1)", ylabel="Ns*(N+1)")
plt.spy(Ax)# ,  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Ax, the A part to the equality constraint")
plt.subplot(2,4,2,xlabel="Ns*(N+1)", ylabel="Nu*N")
plt.imshow(Bu.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Bu, the B part to the equality constraint")
plt.subplot(2,4,3,xlabel="ns*(N+1) + ns*(N+1) + nu*N", ylabel="Ns*(N+1)+Nu*N")
plt.imshow(A.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("A total in $l\\leq Ax \\geq u$")
plt.subplot(2,4,4)
plt.imshow((sparse.kron(sparse.eye(N+1, k=-1), Ad)).toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("A2 in Ax, sparse.kron(sparse.eye(N+1, k=-1), Ad)")
plt.subplot(2,4,5)
plt.imshow(P.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("P in $x^TPx$")


#! Visualize Vectors
plt.subplot(2,4,6)
plt.plot(l)
plt.title('l in  $l\\leq Ax \\geq u$')
plt.grid()
plt.subplot(2,4,7)
plt.plot(u)
plt.title("u")
plt.grid()
plt.subplot(2,4,8)
plt.plot(q)
plt.title("q")
plt.grid()
plt.savefig("Sparse MPC.png")
#plt.show()



# Create an OSQP object
prob = osqp.OSQP()

# Setup workspace
prob.setup(P, q, A, l, u, warm_start=True)

# Simulate in closed loop
nsim = 100

# Store data Init
xst = np.zeros((ns,nsim))
ust = np.zeros((nu,nsim))

t1 = time.clock()
for i in range(nsim):
    # Solve
    res = prob.solve()

    # Check solver status
    if res.info.status != 'solved':
        raise ValueError('OSQP did not solve the problem!')

    # Apply first control input to the plant
    ctrl = res.x[-N*nu:-(N-1)*nu]
    x0 = Ad.dot(x0) + Bd.dot(ctrl)

    # Store Data
    xst[:,i] = x0
    ust[:,i] = ctrl

    # Update initial state
    l[:nx] = -x0
    u[:nx] = -x0
    prob.update(l=l, u=u)

print(time.clock()-t1)
#%% Plots
plt.figure()
for i in range(ns):
  plt.plot(range(nsim),xst[i,:],label=str(i))
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
#plt.show()    
plt.savefig('sim_mcp_quad_pos.png', )


plt.figure()
for i in range(nu):
  plt.plot(range(nsim),ust[i,:],label=str(i))
plt.plot(range(nsim),np.ones(nsim)*umin[1],label='U_{min}',linestyle='dashed', linewidth=1.5, color='black')
plt.plot(range(nsim),np.ones(nsim)*umax[1],label='U_{max}',linestyle='dashed', linewidth=1.5, color='black')
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
#plt.show()  
plt.savefig('sim_mcp_quad_u.pdf',format='pdf', dpi=1200) 

#%%
