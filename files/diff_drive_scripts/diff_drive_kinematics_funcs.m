# Differential drive kinematics model
wheel_radius = 0.0625;
wheel_track = 0.445208;

# initial conditions
x_0 = 0.0;
y_0 = 0.0;
theta_0 = 0.0;

# Inverse Kinematics

vel_x_desired = 1;
vel_y_desired = 0;
theta_desired = atan2(vel_y_desired,vel_x_desired);
omega_z_desired = 0;

joints = [
  (1/(wheel_radius*cos(theta_desired))), (wheel_track/(2.0*wheel_radius)); # RIGHT JOINT
  (1/(wheel_radius*cos(theta_desired))), -(wheel_track/(2.0*wheel_radius)) # LEFT JOINT
]* [vel_x_desired; omega_z_desired]
