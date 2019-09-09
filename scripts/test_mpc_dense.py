#%% Initial Data

import osqp
import numpy as np
import scipy as sp
import scipy.sparse as sparse
import matplotlib.pyplot as plt
import time
import matplotlib.cm as cm


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
u0 = 10.5916 # Hover Thrust
umin = np.ones(nu)*9.6 - u0
umax = np.ones(nu)*11.0 - u0
xmin = np.array([-np.pi/6,-np.pi/6,-np.inf,-np.inf,-np.inf,-1.,
                 -np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf])
xmax = np.array([ np.pi/6, np.pi/6, np.inf, np.inf, np.inf, np.inf,
                  np.inf, np.inf, np.inf, np.inf, np.inf, np.inf])

# Sizes
ns = 12
nu = 4

# Objective function
Q = sparse.diags([0., 0., 10., 10., 10., 10., 0., 0., 0., 5., 5., 5.])
QN = Q
R = sparse.eye(nu)

#%% 

def block_diag(M,n):
  """bd creates a sparse block diagonal matrix by repeating M n times
  
  Args:
      M (2d numpy array): matrix to be repeated
      n (float): number of times to repeat
  """
  return sparse.block_diag([M for i in range(n)])

# Prediction horizon
N = 100

# Initial and reference states
x0 = np.zeros(ns)
xr = np.array([0.,0.,4.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
xrtrj = np.kron(np.ones(N), xr.reshape(xr.shape[0],-1))

# Plot Ad and Bd
""" plt.figure()
plt.subplot(1,2,1,xlabel="Ns", ylabel="Ns")
plt.imshow(Ad.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Ad in $x_{k+1}=Adx_k+Bu_k$")
plt.subplot(1,2,2,xlabel="Nu", ylabel="Ns")
plt.imshow(Bd.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Bdd in $x_{k+1}=Adx_k+Bdu_k$")
plt.show() """


#! Cast MPC problem to a QP: x = [u(0),...,u(N-1)) DENSE FORM
# - quadratic objective, use 
Rbd = sparse.kron(sparse.eye(N), R)
Qbd = sparse.kron(sparse.eye(N), Q)
Bbd = block_diag(Bd,nu)

""" plt.figure()
plt.subplot(1,1,1,xlabel="Ns", ylabel="Ns")
plt.imshow(Bbd.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Block diagonal B")
plt.show() """

# Write B:
diag_AkB = Bd
data_list = Bbd.data
row_list = Bbd.row
col_list = Bbd.col
B = sparse.coo_matrix
for i in range(N):
    if i<N-1:
        AkB_bd_temp = block_diag(diag_AkB,N-i)
    else:
        AkB_bd_temp = diag_AkB.tocoo()
    data_list = np.hstack([data_list,AkB_bd_temp.data])
    row_list  = np.hstack([row_list,AkB_bd_temp.row+np.full((AkB_bd_temp.row.shape[0],),ns*i)])
    col_list  = np.hstack([col_list,AkB_bd_temp.col])

    diag_AkB = Ad.dot(diag_AkB)
    #diag_AkB = diag_AkB + [Ad.dot(diag_AkB[-1])]
    
"""     plt.figure()
    plt.subplot(1,1,1,xlabel="Ns", ylabel="Ns")
    plt.imshow(AkB_bd_temp.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
    plt.title("Block diagonals of B")
    plt.show() """

B = sparse.coo_matrix((data_list, (row_list, col_list)), shape=(N*ns, N*nu))
#B = sparse.diags(diag_AkB,[-5,-4,-3,-2,-1,0])
""" plt.figure()
plt.subplot(1,1,1,xlabel="Ns", ylabel="Ns")
plt.imshow(B.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("B")
plt.show() """

a = Ad
Ak = Ad
for i in range(N-1):
    Ak = Ad.dot(Ad)
    a = sparse.vstack([a,Ak])

#a = sparse.csc_matrix(a)


#plt.subplot(1,1,1,xlabel="Ns", ylabel="Ns")
""" plt.matshow(a.toarray(), cmap=cm.jet) 
plt.title("a")
plt.colorbar()
plt.show() """


P = Rbd + B.T @ Qbd @ B
# - linear objective
xrQB  = B.T @ np.kron(np.ones(N), Q.dot(xr))
x0_1 = x0.reshape(x0.shape[0],-1)
x0aQb = ( (x0_1.T)@(a.T)@(Qbd)@(B) ).squeeze()

""" plt.figure()
plt.subplot(1,1,1,xlabel="Ns", ylabel="Ns")
plt.matshow(xrQB,  interpolation='nearest', cmap=cm.Greys_r)
plt.title("xrQB")
plt.show() """

q = np.zeros((N*nu))#np.hstack([x0aQb - xrQB]) #np.reshape( x0aQb - xrQB ,(N*nu,)) #,order='F')

# - input and state constraints
Aineq_u = sparse.eye(N*nu)
Aineq_x   = B
l = [] #np.hstack([np.kron(np.ones(N), xmin)-a.dot(x0), np.kron(np.ones(N), umin)])
u = [] #np.hstack([np.kron(np.ones(N), xmax)-a.dot(x0), np.kron(np.ones(N), umax)])
# - OSQP constraints
A = [] # sparse.vstack([Aineq_x, Aineq_u]).tocsc()

# Create an OSQP object
prob = osqp.OSQP()


#! Visualize Matrices
fig = plt.figure()

""" fig.suptitle("QP Matrices to solve MP in dense form. N={}, ns={}, nu={}".format(N,ns,nu),fontsize=20)
plt.subplot(2,4,1,xlabel="Ns*(N+1)", ylabel="Ns*(N+1)")
plt.imshow(a.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("a in $x=ax_0+bu$")
plt.subplot(2,4,2,xlabel="Ns*(N+1)", ylabel="Nu*N")
plt.imshow(B.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("b in $x=ax_0+bu$")
plt.subplot(2,4,3,xlabel="ns*(N+1) + ns*(N+1) + nu*N", ylabel="Ns*(N+1)+Nu*N")
plt.imshow(A.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("A total in $l\\leq Ax \\geq u$")
plt.subplot(2,4,4)
plt.imshow(P.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("P in $J=u^TPu+q^Tu$")
plt.subplot(2,4,5)
plt.imshow(Qbd.toarray(),  interpolation='nearest', cmap=cm.Greys_r)
plt.title("Qbd")


#! Visualize Vectors
plt.subplot(2,4,6)
plt.plot(l)
plt.title('l in  $l\\leq Ax \\geq u$')
plt.grid()
plt.subplot(2,4,7)
plt.plot(u)
plt.title("l in  $l\\leq Ax \\geq u$")
plt.grid()
plt.subplot(2,4,8)
plt.plot(q)
plt.title("q in $J=u^TPu+q^Tu$")
plt.grid()
plt.tight_layout()
plt.savefig("Sparse MPC.png",bbox_inches='tight')
plt.show() """



# Setup workspace
prob.setup(P, warm_start=True)




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
    ctrl = res.x[:nu]
    x0 = Ad.dot(x0) + Bd.dot(ctrl)

    # Store Data
    xst[:,i] = x0
    ust[:,i] = ctrl

    # Update initial state
    #l[:nx] = -x0
    q = 
    prob.update(q=q)

print(time.clock()-t1)
#%% Plots
for i in range(ns):
  plt.plot(range(nsim),xst[i,:],label=str(i))
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
plt.show()    
plt.savefig('sim_mcp_quad_pos_dense.png')


for i in range(nu):
  plt.plot(range(nsim),ust[i,:],label=str(i))
plt.plot(range(nsim),np.ones(nsim)*umin[1],label='U_{min}',linestyle='dashed', linewidth=1.5, color='black')
plt.plot(range(nsim),np.ones(nsim)*umax[1],label='U_{max}',linestyle='dashed', linewidth=1.5, color='black')
plt.xlabel('Time(s)')
plt.grid()
plt.legend()
plt.show()  
plt.savefig('sim_mcp_quad_u.png')

#%%
