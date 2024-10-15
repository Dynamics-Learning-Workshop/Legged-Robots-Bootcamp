function [t, y] = my_ode45(odefun, tspan, y0, RelTol, AbsTol)
    % Initialization
    t = tspan(1);
    y = y0;
    h = initial_step_size; % Define an initial step size
    max_step = tspan(2);
    
    while t < max_step
        % Calculate the next step using a method (e.g., RK4)
        [y_new, error_estimate] = runge_kutta_step(odefun, t, y, h);
        
        % Calculate tolerances
        rel_error = abs(error_estimate) ./ max(abs(y_new), AbsTol);
        if any(rel_error > RelTol)
            % Adjust step size based on error
            h = h * 0.9 * (RelTol ./ rel_error) .^ (1/4); % Example step size adjustment
        else
            % Accept the step
            t = t + h;
            y = y_new;
            % Update the step size for the next iteration
            h = h * 1.1; % Increase step size if the previous step was accepted
        end
        
        % Ensure step size does not exceed the maximum allowable step
        h = min(h, max_step - t);
    end
end
