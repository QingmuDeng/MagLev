import numpy as np
import matplotlib.pyplot as plt

## Difference Equation
def diff_eqn_josh():
    return 1/(L*m)*(
        k1*v[-1]*dt**3
        -z[-1]*(-3*L*m+R*m*dt)
        -z[-2]*(3*L*m-2*R*m-k2*L*dt**2)
        -z[-3]*(L*m+R*m*dt+k2*L*dt**2-k2*R*dt**3)
    )

# Parameters
L = 1e-3 # coil inductance (Henry)
m = 0.05 # magnet mass (kg)
R = 2 # coil resistance (Ohm)
dt = 0.001 # Time step (s)
k1 = 1 # electrical constant
k2 = 1 # positional constant
v = [10] # Drive voltage (V)
z0 = -0.00000005 # equilibrium position (m)
z = [z0, z0, z0] # position array

## Simulating
t = [0, dt, 2*dt]
for i in range(50):
    t.append((3+i)*dt)
    z.append(diff_eqn_josh())

fig, ax = plt.subplots()
ax.plot(t, z)
plt.show()
