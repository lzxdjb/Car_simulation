import tinympc
import numpy as np

# Define necessary data
A = np.array([ # 
    [1.0, 0.0, 0.0, 0.0, 0.0245250, 0.0, 0.050, 0.0, 0.0, 0.0, 0.02044, 0.0],
    [0.0, 1.0, 0.0, -0.0245250, 0.0, 0.0, 0.0, 0.050, 0.0, -0.02044, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.050, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0250000, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0250000, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.025],
    [0.0, 0.0, 0.0, 0.0, 0.9810000, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0122625, 0.0],
    [0.0, 0.0, 0.0, -0.9810000, 0.0, 0.0, 0.0, 1.0, 0.0, -0.0122625, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])

B = np.array([ # 
    [-0.07069, 0.07773, 0.07091, -0.07795],
    [0.07034, 0.07747, -0.07042, -0.07739],
    [0.0052554, 0.0052554, 0.0052554, 0.0052554],
    [-0.1720966, -0.1895213, 0.1722891, 0.1893288],
    [-0.1729419, 0.1901740, 0.1734809, -0.1907131],
    [0.0123423, -0.0045148, -0.0174024, 0.0095748],
    [-0.0565520, 0.0621869, 0.0567283, -0.0623632],
    [0.0562756, 0.0619735, -0.0563386, -0.0619105],
    [0.2102143, 0.2102143, 0.2102143, 0.2102143],
    [-13.7677303, -15.1617018, 13.7831318, 15.1463003],
    [-13.8353509, 15.2139209, 13.8784751, -15.2570451],
    [0.9873856, -0.3611820, -1.3921880, 0.7659845]])

Q = np.diag([1.0, 1.0, 1.0, 0.4, 0.4, 0.4, # 
            0.4, 0.4, 0.4, 0.2, 0.2, 0.2]);
R = np.diag([1000.0]*4); # 

N = 20 # 

# Set up the problem
prob = tinympc.TinyMPC()
prob.setup(A, B, Q, R, N)

# Define initial condition
x0 = np.array([0.5, 1.3, -0.7, 0, 0, 0, 0, 0, 0, 0, 0, 0])

# Set the first state in the horizon
prob.set_x0(x0)

# Solve the problem
solution = prob.solve()

# Print the controls at the first time step
print(solution["controls"])

# Simulate for an arbitrary number of time steps
Nsim = 350
xs = np.zeros((Nsim, Q.shape[0])) # History of states for plotting
us = np.zeros((Nsim, R.shape[0])) # History of controls for plotting
for i in range(Nsim):
    prob.set_x0(x0) # Set the first state in the horizon
    solution = prob.solve() # Solve the problem
    x0 = A@x0 + B@solution["controls"] # Simulate the system 
    xs[i] = x0
    us[i] = solution["controls"]

import matplotlib.pyplot as plt

# Plot trajectory
fig, axs = plt.subplots(2, 1, sharex=True)
axs[0].plot(xs[:,:3], label=["x", "y", "z"])
axs[1].plot(us, label=["u1", "u2", "u3", "u4"])
axs[0].set_title("quadrotor trajectory over time")
axs[1].set_xlabel("time steps (100Hz)")
axs[0].legend()
axs[1].legend()
plt.show()