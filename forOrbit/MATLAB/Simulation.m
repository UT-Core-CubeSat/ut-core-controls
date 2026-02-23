Parameters

% Reference
refGen = ReferenceGenerator(Param);

%% Instantiate Classes
dynamics     = Dynamics(Param);
sensors      = Sensors(Param);
measurements = sensors.measurements(zeros(size(Param.states_init)), Param.states_init, 0);
observer     = Observer(Param);
controller   = Controller(Param);
animation    = Animation(Param);
plotter      = Plotter(Param);

% Focus active figure
list_of_figures   = findobj('type','figure');
number_of_figures = length(list_of_figures);
if Param.active_figure <= number_of_figures
    figure(Param.active_figure);
end

timer = tic;
t = Param.t_start;
t_next_plot = 0;
reference = zeros(10,1);
states_hat = Param.states_init;

while t < Param.t_end

    drawn_recently = false;
    while toc(timer) < t / Param.speed
        if ~drawn_recently
            drawnow
            drawn_recently = true;
        end
    end

    while t <= t_next_plot

        % Reference
        [reference, mode] = refGen.update(dynamics.states, t);

        % Sensors / Observer
        measurements = sensors.measurements(dynamics.states_dot, dynamics.states, t);  

        states_hat = observer.update(measurements);
        %states_hat = dynamics.states;
        % Inputs
        [tau, states_m] = controller.update(states_hat, reference, measurements, mode);

        % Dynamics
        dynamics.update(tau, mode);           
        % Time step
        t = t + Param.Ts;
    end
    
    % Combine the states into a single vector
    states     = dynamics.states;
    estimate   = states_hat;
    references = reference;
    model      = states_m;
    input      = tau;
    % Update animation and plots
    animation.update(states, reference, mode);
    plotter.update(t, states, estimate, references, model, input, measurements, mode);
    % Update time
    t_next_plot = t + Param.t_plot;
end
simulation_length = toc(timer);
fprintf("Simulation took: %f seconds.\n",simulation_length);