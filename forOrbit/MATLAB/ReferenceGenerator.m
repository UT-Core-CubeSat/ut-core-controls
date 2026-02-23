classdef ReferenceGenerator < handle
    properties
        Ts
        mode
        schedule
        Helpers
        DateTime
        currentIndex
        last_q_ref
        initialized
    end

    methods

        function self = ReferenceGenerator(Param)
            self.Ts = Param.Ts;
            self.schedule = Param.schedule;
            self.mode = 'off';
            self.Helpers = HelperFunctions;
            self.DateTime = Param.DateTime;
            self.currentIndex = 1;
            self.last_q_ref = [];
            self.initialized = false;
        end

        function [reference, mode] = update(self, states, t)
            % 1️⃣ Get current schedule entry
            s = self.checkSchedule(t);

            % 2️⃣ Branch by mode
            switch lower(s.mode)
                case 'findj'
                    [q_ref, w_ref, a_ref] = self.findJMode(states, t);

                case 'detumble'
                    %disp("Detumbling...")
                    [q_ref, w_ref, a_ref] = self.detumbleMode(states);

                case 'point'
                    % If target is a string, check keywords; otherwise treat as vector
                    if ischar(s.target) || isstring(s.target)
                        switch lower(s.target)
                            case 'sun'
                                %disp("Sun pointing...")
                                [q_ref, w_ref, a_ref] = self.sunPointMode(t, s.face);
                            case 'nadir'
                                %disp("Pointing Nadir")
                                [q_ref, w_ref, a_ref] = self.nadirPointMode(states, s.face);
                            otherwise
                                %disp("Unknown pointing object")
                                [q_ref, w_ref, a_ref] = self.offMode(states);
                        end
                    else
                        if ~isempty(s.slew)
                            %disp("Slewing")
                            [q_ref, w_ref, a_ref] = self.slewMode(states, s.slew);
                        elseif ~isempty(s.target)
                            [q_ref, w_ref, a_ref] = self.targetPointMode(s.face, s.target);
                        else
                            [q_ref, w_ref, a_ref] = self.offMode();
                        end
                    end

                case 'off'
                    %disp("Off")
                    [q_ref, w_ref, a_ref] = self.offMode(states);

                otherwise
                    %disp("Off")
                    [q_ref, w_ref, a_ref] = self.offMode(states);
            end
            q_ref = self.enforceQuaternionContinuity(q_ref);
            reference = [q_ref; w_ref; a_ref];
            mode = s.mode;
        end

        function [q_ref, w_ref, a_ref] = findJMode(~, states, t)
            % ---------------------------------------------------------------
            % findJMode
            % Generates persistently-exciting reference body rates
            % to identify uncertain inertia via MRAC.
            %
            % Two body axes receive sinusoidal excitation at a time
            % while the quaternion reference remains fixed.
            %
            % Reference signal:
            %   w_ref = A * [ sin(w1 t); sin(w2 t); 0 ]
            % or any axis permutation depending on t.
            %
            % a_ref is analytically computed as the derivative of w_ref.
            % ---------------------------------------------------------------

            % Keep the attitude from drifting
            q_ref = states(7:10);

            % --- Tunable excitation parameters ---
            A  = 0.03;          % rad/s amplitude (gentle!)
            f1 = 0.043;          % Hz for axis 1
            f2 = 0.037;         % Hz for axis 2 (non-harmonic)
            w1 = 2*pi*f1;
            w2 = 2*pi*f2;

            % --- Rotate which 2 axes get excited every N seconds ---
            cycleTime = 100;  % seconds per axis pair
            k = floor(t / cycleTime);

            % default (x–y excitation)
            idx = mod(k,3);

            switch idx
                case 0  % excite x & y
                    w_ref = [ A*sin(w1*t);
                        A*sin(w2*t);
                        0 ];

                    a_ref = [ A*w1*cos(w1*t);
                        A*w2*cos(w2*t);
                        0 ];

                case 1  % excite y & z
                    w_ref = [ 0;
                        A*sin(w1*t);
                        A*sin(w2*t) ];

                    a_ref = [ 0;
                        A*w1*cos(w1*t);
                        A*w2*cos(w2*t) ];

                case 2  % excite z & x
                    w_ref = [ A*sin(w1*t);
                        0;
                        A*sin(w2*t) ];

                    a_ref = [ A*w1*cos(w1*t);
                        0;
                        A*w2*cos(w2*t) ];
            end
        end

        function [q_ref, w_ref, a_ref] = detumbleMode(~,states)
            q_ref = states(7:10);
            w_ref = [0;0;0];
            a_ref = [0;0;0];
        end

        function [q_ref, w_ref, a_ref] = sunPointMode(self, t, face)
            jd = self.Helpers.julianDate(self.DateTime + seconds(t));
            r_sun = self.Helpers.earth2sun(jd);
            bodyAxis = face;
            q_ref = self.Helpers.quatFromTwoVectors(bodyAxis, r_sun);
            w_ref = [0;0;0];
            a_ref = [0;0;0];
        end

        function [q_ref, w_ref, a_ref] = targetPointMode(self, face, target)
            targetECI = target;
            targetECI = targetECI / norm(targetECI);
            bodyAxis = face;
            q_ref = self.Helpers.quatFromTwoVectors(bodyAxis, targetECI);
            w_ref = [0;0;0];
            a_ref = [0;0;0];
        end

        function [q_ref, w_ref, a_ref] = nadirPointMode(self, states, face)
            R = states(1:3);
            r_hat = -R / norm(R);
            bodyAxis = face;
            q_ref = self.Helpers.quatFromTwoVectors(bodyAxis, r_hat);
            w_ref = [0;0;0];
            a_ref = [0;0;0];
        end

        function [q_ref, w_ref, a_ref] = slewMode(~, states, slew)
            q_ref = states(7:10);
            w_ref = slew;
            a_ref = [0;0;0];
        end

        function [q_ref, w_ref, a_ref] = offMode(~, states)
            q_ref = states(7:10);
            w_ref = zeros(3,1);
            a_ref = zeros(3,1);
        end



        function schedule = checkSchedule(self, t)
            % Check current mode window
            s = self.schedule(self.currentIndex);


            % Advance if we passed this segment
            if t > s.t_end && self.currentIndex < length(self.schedule)
                self.currentIndex = self.currentIndex + 1;
                s = self.schedule(self.currentIndex);
            end

            % Pack up
            schedule = s;
        end

        function q_out = enforceQuaternionContinuity(self, q_new)
            % On first call, just store and return
            if ~self.initialized
                self.last_q_ref = q_new;
                self.initialized = true;
                q_out = q_new;
                return;
            end

            % Flip sign if hemisphere mismatch
            if self.last_q_ref' * q_new < 0
                q_new = -q_new;
            end

            % Store and return
            self.last_q_ref = q_new;
            q_out = q_new;
        end


    end

end