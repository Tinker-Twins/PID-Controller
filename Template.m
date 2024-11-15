% Create a PID controller object with specified gains and buffer size
pid_controller = PIDController(0.36, 0.16, 0.14, 50); % kP=0.36, kI=0.16, kD=0.14, kS=50

% Define the error and time
error = 10.0;  % Error term (difference between setpoint and current value)
time = 100.0;  % Current time instant

% Get the control signal (command) from the PID controller
command = pid_controller.control(error, time);  % PID controller output (command)

% Display the result
fprintf('Control Command: %.2f\n', command);