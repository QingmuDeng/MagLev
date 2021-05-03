from simulator import Simulator, DEFAULT_constants, DEFAULT_initial_conditions, DEFAULT_controller_function
import matplotlib.pyplot as plt

simulator = Simulator(DEFAULT_constants, DEFAULT_initial_conditions)
results = simulator.run(dt=0.001, t_final = 5.0, controller_function=DEFAULT_controller_function)

plt.plot(results["t"], results["z_nonlinear"])
plt.plot(results["t"], results["z_linear"])
plt.title("Magnet Position Over Time")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.legend(["Nonlinear Model", "Linearized Model"])
plt.ylim([-0.20, 0.0])
plt.show()
