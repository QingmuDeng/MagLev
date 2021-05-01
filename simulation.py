import numpy as np
import matplotlib.pyplot as plt

## Difference Equation
def diff_eqn_josh():
    z_next = 1/(L*m)*(
        k1*v[-1]*dt**3
        -z[-1]*(-3*L*m+R*m*dt)
        -z[-2]*(3*L*m-2*R*m-k2*L*dt**2)
        -z[-3]*(L*m+R*m*dt+k2*L*dt**2-k2*R*dt**3)
    )
    if t[-1] < 8*dt:
        # print("Time: ",t[-1])
        # print("1/(L*m)*k1*v[-1]*dt**3 : ", 1/(L*m)*k1*v[-1]*dt**3)
        # print("1/(L*m)*-z[-1]*(-3*L*m+R*m*dt) : ", 1/(L*m)*-z[-1]*(-3*L*m+R*m*dt))
        # print("1/(L*m)*(-z[-2])*(3*L*m-2*R*m-k2*L*dt**2) : ", 1/(L*m)*(-z[-2])*(3*L*m-2*R*m-k2*L*dt**2))
        # print("1/(L*m)*(-z[-3])*(L*m+R*m*dt+k2*L*dt**2-k2*R*dt**3) : ", 1/(L*m)*(-z[-3])*(L*m+R*m*dt+k2*L*dt**2-k2*R*dt**3))
        # print("3*L*m-2*R*m: ", 3*L*m-2*R*m)
        # print("1/(L*m): ", 1/(L*m))
        print("z_next: ", z_next)
    if z_next > 0:
        return 0
    elif z_next < - 0.20:
        return - 0.20
    return z_next

def diff_eqn_ever():
    # i(t+dt) = dt*v(t)/L + i(t) - R*i(t)*dt/L
    # z(t+2dt) = dt**2/m*(2*K*i_0*i(t)/z_0**2 - 2*K*i_0**2 * z(t)/z_0**3) + 2z(t+dt) - z(t)
    i_list.append(dt*v[-1]/L + i_list[-1] - R*i_list[-1]*dt/L)
    z_next = dt**2/m*(2*K*i_0*i_list[-2]/z_0**2 - 2*K*i_0**2 * z[-2]/z_0**3) + 2*z[-1] - z[-2]
    if z_next > 0:
        z_next = 0
    elif z_next < - 0.20:
        z_next = - 0.20
    # print("Current: ", i_list[-1], " | Position: ", z_next)
    return z_next

# Parameters
L = 0.4125 # coil inductance (Henry)
m = 0.068 # magnet mass (kg)
R = 10 # coil resistance (Ohm)
dt = 0.00001 # Time step (s)
i_0 = 0.8 # steady current
z_0 = -0.0012 # steady pos
K = 6.53e-5 #constant
k1 = 1 # electrical constant
k2 = 1 # positional constant
v = [R*i_0] # Drive voltage (V)
i_list = [0, 0]
# z0 = -0.00000000000001 # equilibrium position (m)
z = [z_0, z_0, z_0] # position array

# Print out forces
print("gravity: ", m*9.8,"\t magnetic: ", K*i_0**2/z_0**2)


## Simulating
t = [0, dt, 2*dt]
for i in range(5000):
    t.append((3+i)*dt)
    z.append(diff_eqn_ever())

fig, ax = plt.subplots()
ax.plot(t, z)
plt.show()
