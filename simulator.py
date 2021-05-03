import numpy as np

DEFAULT_constants = {
    "L" : 0.4125, # coil inductance (Henry)
    "m" : 0.068, # magnet mass (kg)
    "R" : 10, # coil resistance (Ohm)
    "K" : 6.53e-5, #constant
    "g" : 9.81 # acceleration due to gravity
}

DEFAULT_initial_conditions = {
    "i" : 0.8, # steady current
    "z" : -0.175, # steady pos
    "z_dot" : 0.0, # velocity
    "v" : 0.0 # input voltage
}

def DEFAULT_controller_function( input_z ):
    return 0.0

class Simulator:
    def __init__(self, constants, initial_conditions):
        self.const = constants
        self.list_i = [ initial_conditions["i"] ] * 3
        self.list_linear_z = [ initial_conditions["z"] ] * 3
        self.list_nonlinear_z = [ initial_conditions["z"] ] * 3
        self.list_nonlinear_zdot = [ initial_conditions["z_dot"] ] * 3
        self.list_nonlinear_zdotdot = [0.0] * 3
        self.list_v = [ initial_conditions["v"] ] * 3

    def update_i(self):
        next_i = self.dt*self.list_v[-1]/self.const["L"] + self.list_i[-1] - self.const["R"]*self.list_i[-1]*self.dt/self.const["L"]
        self.list_i.append(next_i)

    def update_linear_z(self):
        next_z = self.dt**2/self.const["m"]*(2*self.const["K"]*self.list_i[0]*self.list_i[-2]/self.list_linear_z[0]**2 - 2*self.const["K"]*self.list_i[0]**2 * self.list_linear_z[-2]/self.list_linear_z[0]**3) + 2*self.list_linear_z[-1] - self.list_linear_z[-2]
        self.list_linear_z.append(next_z)

    def update_nonlinear_z(self):
        if self.list_nonlinear_z[-1] != 0.0:
            next_zdotdot = ((self.const["K"]/self.const["m"]) * (self.list_i[-1]**2) / (self.list_nonlinear_z[-1]**2)) - self.const["g"]
            self.list_nonlinear_zdotdot.append( next_zdotdot )
            next_zdot = self.list_nonlinear_zdotdot[-1]*self.dt + self.list_nonlinear_zdot[-1]
            self.list_nonlinear_zdot.append( next_zdot )
            z_next = self.list_nonlinear_zdot[-1]*self.dt + self.list_nonlinear_z[-1]
            if z_next > 0:
                z_next = 0
            elif z_next < - 0.20:
                z_next = - 0.20
        else:
            z_next = 0.0
        self.list_nonlinear_z.append(z_next)

    def run(self, dt, t_final, controller_function):
        self.dt = dt
        list_t_post = list(np.linspace(0, t_final, round(t_final/dt) + 1))
        for t in list_t_post:
            self.list_v.append( controller_function(self.list_linear_z[-1]) )
            self.update_i()
            self.update_linear_z()
            self.update_nonlinear_z()
        list_t_pre = [ - list_t_post[3], - list_t_post[2], - list_t_post[1] ]
        self.list_t = list_t_pre + list_t_post
        results = {
            "t" : self.list_t,
            "v" : self.list_v,
            "i" : self.list_i,
            "z_linear" : self.list_linear_z,
            "z_nonlinear" : self.list_nonlinear_z,
            "zdot_nonlinear" : self.list_nonlinear_zdot,
            "zdotdot_nonlinear" : self.list_nonlinear_zdotdot
        }
        return results
