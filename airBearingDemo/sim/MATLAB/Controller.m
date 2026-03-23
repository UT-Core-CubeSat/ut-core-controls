classdef Controller < handle
    properties
        % Controllers
        NDI
        Bdot

        % Time step
        Ts

        % Previous mode 
        prev_mode
    end

    methods
        function self = Controller(Param)
            % Instantiate both controllers
            self.NDI = ControllerNDI(Param);
            self.Bdot = ControllerBdot(Param); % already in your project

            self.Ts = Param.Ts;

            self.prev_mode = '';
        end

        function [tau, states_m] = update(self, states_hat, reference, measurements, mode)           
            
            % Run appropriate control law
            switch lower(mode)
                case {'findj'}
                    [tau_rw, tau_desat, states_m] = self.NDI.update(states_hat, reference, measurements);
                    tau = [tau_rw;tau_desat];
                case {'point'}
                    % Fine Pointing? NDI.
                    [tau_rw, tau_desat, states_m] = self.NDI.update(states_hat, reference, measurements);
                    tau = [tau_rw;tau_desat];
                    
                case {'detumble'}
                    % Detumble? BDot.
                    tau_mtq = self.Bdot.update(measurements);
                    states_m = states_hat(7:13);
                    tau = [zeros(4,1);tau_mtq];

                case {'off'}
                    % Everything off
                    tau = zeros(7,1);
                    states_m = states_hat(7:13);

                otherwise
                    % If mode is unknown, turn everything off anyway
                    warning("Unknown controller mode '%s', switching OFF", mode);
                    tau = zeros(7,1);
                    states_m = states_hat(7:13);
            end
        end
    end
end
