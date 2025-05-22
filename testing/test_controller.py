import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters
dt = 0.1  # Time step (10Hz)
sim_time = 5.0  # Simulation duration (seconds)
n_steps = int(sim_time / dt) + 1
time = np.linspace(0, sim_time, n_steps)

# System limits
max_acc = 0.5  # Maximum acceleration
max_vel = 1.0  # Maximum velocity

# PD controller gains
Kp_vel = 5.0  # Proportional gain for velocity controller
Kd_vel = 0.55  # Derivative gain for velocity controller
Kp_pos = 2.0  # Proportional gain for position controller
Kd_pos = 0.25  # Derivative gain for position controller

# Simulator class
class ActuatorSimulator:
    def __init__(self, dt, max_acc, max_vel):
        self.dt = dt
        self.max_acc = max_acc
        self.max_vel = max_vel
        self.pos = 0.0
        self.vel = 0.0

    def update(self, acc):
        # Apply acceleration limits
        acc = np.clip(acc, -self.max_acc, self.max_acc)
        
        # Update velocity
        self.vel += acc * self.dt
        self.vel = np.clip(self.vel, -self.max_vel, self.max_vel)
        
        # Update position
        self.pos += self.vel * self.dt
        
        return self.pos, self.vel

# PD controller class
class PDController:
    def __init__(self, Kp, Kd, dt):
        self.Kp = Kp
        self.Kd = Kd
        self.dt = dt
        self.prev_error = 0.0

    def compute(self, ref, actual):
        error = ref - actual
        error_deriv = (error - self.prev_error) / self.dt
        self.prev_error = error
        return self.Kp * error + self.Kd * error_deriv

# Run velocity-only tracking simulation
def run_velocity_tracking():
    sim = ActuatorSimulator(dt, max_acc, max_vel)
    vel_controller = PDController(Kp_vel, Kd_vel, dt)
    
    vel_ref = 0.8  # Target velocity
    positions = np.zeros(n_steps)
    velocities = np.zeros(n_steps)
    vel_refs = np.full(n_steps, vel_ref)
    
    for i in range(n_steps):
        acc = vel_controller.compute(vel_ref, sim.vel)
        pos, vel = sim.update(acc)
        positions[i] = pos
        velocities[i] = vel
    
    return positions, velocities, vel_refs

# Run position tracking simulation with cascaded controller
def run_position_tracking():
    sim = ActuatorSimulator(dt, max_acc, max_vel)
    pos_controller = PDController(Kp_pos, Kd_pos, dt)
    vel_controller = PDController(Kp_vel, Kd_vel, dt)
    
    pos_ref = 1.0  # Target position
    positions = np.zeros(n_steps)
    velocities = np.zeros(n_steps)
    pos_refs = np.full(n_steps, pos_ref)
    vel_refs = np.zeros(n_steps)
    
    for i in range(n_steps):
        vel_ref = pos_controller.compute(pos_ref, sim.pos)
        acc = vel_controller.compute(vel_ref, sim.vel)
        pos, vel = sim.update(acc)
        positions[i] = pos
        velocities[i] = vel
        vel_refs[i] = vel_ref
    
    return positions, velocities, pos_refs, vel_refs

# Run simulations
vel_pos, vel_vel, vel_refs = run_velocity_tracking()
pos_pos, pos_vel, pos_refs, pos_vel_refs = run_position_tracking()

# Output max position for overshoot
print("Maximum position (overshoot):", np.max(pos_pos))


# Plot results
plt.figure(figsize=(12, 8))

# Velocity tracking plots
plt.subplot(2, 2, 1)
plt.plot(time, vel_refs, 'r--', label='Reference')
plt.plot(time, vel_vel, 'b-', label='Actual')
plt.title('Velocity Tracking')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 2)
plt.plot(time, vel_pos, 'b-', label='Position')
plt.title('Position During Velocity Tracking')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.grid(True)
plt.legend()

# Position tracking plots
plt.subplot(2, 2, 3)
plt.plot(time, pos_refs, 'r--', label='Reference')
plt.plot(time, pos_pos, 'b-', label='Actual')
plt.title('Position Tracking')
plt.xlabel('Time (s)')
plt.ylabel('Position')
plt.grid(True)
plt.legend()

plt.subplot(2, 2, 4)
plt.plot(time, pos_vel_refs, 'r--', label='Reference')
plt.plot(time, pos_vel, 'b-', label='Actual')
plt.title('Velocity During Position Tracking')
plt.xlabel('Time (s)')
plt.ylabel('Velocity')
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig('pd_controller_results.png')