# Trapezoidal trajectory generator (WIP)
Generate trajectories with trapezoidal velocity shape. 

- Initialize the generator position (`position initial`), maximum velocity (`velocity_max`) and maximum acceleration (`acceleration_max`)
- Configure new setpoint `def set_setpoint(self, setpoint, velocity_initial, velocity_final=0)`. You have an option for non-zero final velocity.
- Call `def update_step(self):` to generate next position. (Should be called at fixed rate)



