classdef PIDController
    % PIDController: A class that implements a PID controller with error history buffering.
    % It uses proportional, integral, and derivative control actions.
    
    properties
        kP         % Proportional gain
        kI         % Integral gain
        kD         % Derivative gain
        kS         % Saturation constant (history buffer size)
        err_int    % Accumulated error (integral)
        err_dif    % Error difference (derivative)
        err_prev   % Previous error
        err_hist   % FIFO buffer for error history
        t_prev     % Previous time (timestamp)
    end
    
    methods
        % Constructor to initialize the PID controller with the specified gains
        function obj = PIDController(kP, kI, kD, kS)
            obj.kP = kP;        % Set proportional gain
            obj.kI = kI;        % Set integral gain
            obj.kD = kD;        % Set derivative gain
            obj.kS = kS;        % Set saturation constant (error history buffer size)
            obj.err_int = 0;    % Initialize integral error term
            obj.err_dif = 0;    % Initialize derivative error term
            obj.err_prev = 0;   % Initialize previous error term
            obj.err_hist = [];  % Initialize error history (empty)
            obj.t_prev = 0;     % Initialize previous time
        end
        
        % Method to compute the PID control output
        function u = control(obj, err, t)
            % err: Current error (instantaneous error with respect to setpoint)
            % t  : Current timestamp
            
            dt = t - obj.t_prev;  % Time difference (delta time)
            
            if dt > 0.0
                % Update error history (FIFO buffer logic)
                obj.err_hist = [obj.err_hist, err];  % Add current error to history
                
                % Jacketing to prevent integral windup by removing the oldest error if buffer is full
                if length(obj.err_hist) > obj.kS
                    obj.err_int = obj.err_int - obj.err_hist(1);  % Remove the oldest error from integral
                    obj.err_hist = obj.err_hist(2:end);  % Remove oldest error (FIFO behavior)
                end
                
                % Update integral term (accumulated error)
                obj.err_int = obj.err_int + err;
                
                % Calculate the error difference (derivative)
                obj.err_dif = err - obj.err_prev;
                
                % Apply PID control law
                u = (obj.kP * err) + (obj.kI * obj.err_int * dt) + (obj.kD * obj.err_dif / dt);
                
                % Update the previous error and timestamp for the next iteration
                obj.err_prev = err;
                obj.t_prev = t;
                
                return;
            end
            
            u = 0;  % Default output if dt <= 0 (invalid time step)
        end
    end
end