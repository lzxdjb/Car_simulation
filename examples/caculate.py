
import numpy as np
x_final = np.array([[3.5, 2.7 , 0.5]] )
# A = np.identity(state_size)

Q = np.array([[1.0, 0.0, 0.0], [0.0, 5.0, 0.0], [0.0, 0.0, 0.1]])

print(x_final @ Q)