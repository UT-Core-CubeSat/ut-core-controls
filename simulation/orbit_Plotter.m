classdef orbit_Plotter < handle
    properties
        % History buffers
        time_history
        state_history
        estim_history
        refer_history
        model_history
        input_history
        measurements_history

        % Flags
        plot_translation
        plot_orientation
        plot_input
        plot_RPM
        plot_reference
        plot_estimate
        plot_model
        plot_measurements

        % Handles (line objects)
        handles

        % Indexes
        index

        % Mode text
        legend_text_handle
    end

    methods
        function self = orbit_Plotter(Param)
            self.plot_translation  = Param.plot_translation;
            self.plot_orientation  = Param.plot_orientation;
            self.plot_input        = Param.plot_input;
            self.plot_RPM          = Param.plot_RPM;
            self.plot_reference    = Param.plot_reference;
            self.plot_estimate     = Param.plot_estimate;
            self.plot_model        = Param.plot_model;
            self.plot_measurements = Param.plot_measurements;

            N = ceil((Param.t_end-Param.t_start)/Param.t_plot);
            self.time_history         = NaN(1,  N);
            self.state_history        = NaN(17, N);
            self.estim_history        = NaN(17, N);
            self.refer_history        = NaN(10, N);
            self.model_history        = NaN(7, N);
            self.input_history        = NaN(7,  N);
            self.measurements_history = NaN(29, N);
            self.index = 1;

            self.buildFigures(Param);
        end

        function buildFigures(self, Param)

            t = self.time_history;
            rgb = {'r','g','b'};

            %% Translation (ECI)
            if self.plot_translation
                figure('Windowstyle','docked','name','ECI Translation');
                clf;
                sgtitle('ECI Position and Velocity');

                ax1 = subplot(2,1,1); hold on; grid on;
                for i = 1:3
                    self.handles(i) = plot(t, self.state_history(i,:), ...
                        'Color', rgb{i}, 'LineWidth', Param.line_width, ...
                        'DisplayName', sprintf('r_%c', 'x'+(i-1)));
                end
                ylabel('$r\; (m)$','Interpreter','latex');
                legend(ax1, {'x','y','z'}, 'Location','best');

                ax2 = subplot(2,1,2); hold on; grid on;
                for i = 1:3
                    self.handles(3+i) = plot(t, self.state_history(3+i,:), ...
                        'Color', rgb{i}, 'LineWidth', Param.line_width, ...
                        'DisplayName', sprintf('v_%c', 'x'+(i-1)));
                end
                ylabel('$v\; (m/s)$','Interpreter','latex');
                xlabel('t - time (s)');
                legend(ax2, {'x','y','z'}, 'Location','best');
            end

            %% Orientation
            if self.plot_orientation
                figure('Windowstyle','docked','name','Orientation');
                clf;
                sgtitle('Orientation')

                subplot(7,1,1); hold on; grid on;
                if self.plot_reference; self.handles(25) = plot(self.time_history, self.refer_history(1,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(26) = plot(self.time_history, self.model_history(1,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(27) = plot(self.time_history, self.estim_history(7,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(28) = plot(self.time_history, self.state_history(7,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_0$','Interpreter','latex')

                subplot(7,1,2); hold on; grid on;
                if self.plot_reference; self.handles(29) = plot(self.time_history, self.refer_history(2,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(30) = plot(self.time_history, self.model_history(2,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(31) = plot(self.time_history, self.estim_history(8,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(32) = plot(self.time_history, self.state_history(8,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_1$',"Interpreter","latex")

                subplot(7,1,3); hold on; grid on;
                if self.plot_reference; self.handles(33) = plot(self.time_history, self.refer_history(3,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(34) = plot(self.time_history, self.model_history(3,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(35) = plot(self.time_history, self.estim_history(9,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(36) = plot(self.time_history, self.state_history(9,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_2$',"Interpreter","latex")

                subplot(7,1,4); hold on; grid on;
                if self.plot_reference; self.handles(37) = plot(self.time_history, self.refer_history(4,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(38) = plot(self.time_history, self.model_history(4,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(39) = plot(self.time_history, self.estim_history(10,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(40) = plot(self.time_history, self.state_history(10,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_3$',"Interpreter","latex")

                subplot(7,1,5); hold on; grid on;
                if self.plot_reference; self.handles(41) = plot(self.time_history, self.refer_history(5,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(42) = plot(self.time_history, self.model_history(5,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(43) = plot(self.time_history, self.estim_history(11,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(44) = plot(self.time_history, self.state_history(11,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_x$ (deg/s)','Interpreter','latex')

                subplot(7,1,6); hold on; grid on;
                if self.plot_reference; self.handles(45) = plot(self.time_history, self.refer_history(6,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(46) = plot(self.time_history, self.model_history(6,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(47) = plot(self.time_history, self.estim_history(12,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(48) = plot(self.time_history, self.state_history(12,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_y$ (deg/s)','Interpreter','latex')

                ax = subplot(7,1,7); hold on; grid on;
                if self.plot_reference; self.handles(49) = plot(self.time_history, self.refer_history(7,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(50) = plot(self.time_history, self.model_history(7,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(51) = plot(self.time_history, self.estim_history(13,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(52) = plot(self.time_history, self.state_history(13,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_z$ (deg/s)','Interpreter','latex')
                xlabel("t - time (s)");

                legend_lines = self.handles(25:28);
                legend_handle = legend(legend_lines([self.plot_reference,self.plot_model,self.plot_estimate,true]),"Orientation","horizontal","FontSize",10);
                legend_handle.Position(1:2) = ax.Position(1:2)+[0.15,-0.1];
            end

            %% Inputs
            if self.plot_input
                figure('Windowstyle','docked','name','Inputs (RW)');
                clf; sgtitle('Inputs')
                subplot(4,1,1); hold on; grid on;
                self.handles(53) = plot(self.time_history, self.input_history(1,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_1$ (Nm)','Interpreter','latex')
                subplot(4,1,2); hold on; grid on;
                self.handles(54) = plot(self.time_history, self.input_history(2,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_2$ (Nm)','Interpreter','latex')
                subplot(4,1,3); hold on; grid on;
                self.handles(55) = plot(self.time_history, self.input_history(3,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_3$ (Nm)','Interpreter','latex')
                subplot(4,1,4); hold on; grid on;
                self.handles(56) = plot(self.time_history, self.input_history(4,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_4$ (Nm)','Interpreter','latex'); xlabel("t - time (s)")

                figure('Windowstyle','docked','name','Inputs (MTQ)');
                clf; sgtitle('Inputs')
                subplot(3,1,1); hold on; grid on;
                self.handles(57) = plot(self.time_history, self.input_history(5,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_1$ (Nm)','Interpreter','latex')
                subplot(3,1,2); hold on; grid on;
                self.handles(58) = plot(self.time_history, self.input_history(6,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_2$ (Nm)','Interpreter','latex')
                subplot(3,1,3); hold on; grid on;
                self.handles(59) = plot(self.time_history, self.input_history(7,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_3$ (Nm)','Interpreter','latex'); xlabel("t - time (s)")
            end

            %% Reaction wheel RPMs
            if self.plot_RPM
                figure('Windowstyle','docked','name','RW Speeds');
                clf; hold on; grid on;
                sgtitle('Reaction Wheel Speeds');
                colors = {'r','g','b','k'};
                for i = 1:4
                    self.handles(59+i) = plot(self.time_history, ...
                        self.state_history(13+i,:) * (60/(2*pi)), ...
                        'Color', colors{i}, 'LineWidth', Param.line_width, ...
                        'DisplayName', sprintf('Wheel %d', i));
                end
                xlabel("t - time (s)"); ylabel("Wheel Speed (RPM)");
                legend('show','Location','best');
            end

            %% Measurements
            if self.plot_measurements
                figure('Windowstyle','docked','name','Sensors');
                clf;
                sgtitle('Measurements')

                subplot(2,3,1); hold on; grid on;
                for i = 1:3
                    self.handles(63+i) = plot(t, self.measurements_history(i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Accelerometers'); ylabel("$m/s^2$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                subplot(2,3,2); hold on; grid on;
                for i = 1:3
                    self.handles(66+i) = plot(t, self.measurements_history(3+i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Gyros'); ylabel("$rad/s$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                subplot(2,3,3); hold on; grid on;
                stc = {'r','g','b','k'};
                for i = 1:4
                    self.handles(69+i) = plot(t, self.measurements_history(6+i,:), stc{i}, 'LineWidth',Param.line_width);
                end
                title('Star Tracker'); xlabel("t (s)"); legend({'q0', 'q1', 'q2', 'q3'},'Location','best')

                subplot(2,3,4); hold on; grid on;
                sunc = {'r','g','b','k',[0.5 0.5 0.5],[0.9 0.4 0.1]'};
                for i = 1:6
                    self.handles(73+i) = plot(t, self.measurements_history(10+i,:), 'LineWidth',Param.line_width, 'Color', sunc{i});
                end
                title('Sun Sensors'); ylabel("$I (A)$","Interpreter","latex"); xlabel("t (s)"); legend({'+x','+y','+z','-x','-y','-z'},'Location','best')

                subplot(2,3,5); hold on; grid on;
                for i = 1:3
                    self.handles(79+i) = plot(t, self.measurements_history(16+i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Magnetometers'); ylabel("$T$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                subplot(2,3,6); hold on; grid on;
                wtc = {'r','g','b','k'};
                for i = 1:4
                    self.handles(82+i) = plot(t, self.measurements_history(25+i,:), wtc{i}, 'LineWidth',Param.line_width);
                end
                title('Wheel Tachometers'); ylabel("$RPM$","Interpreter","latex"); xlabel("t (s)"); legend({'Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4'},'Location','best')
            end
            
        end

        function update(self, t, state, estimate, reference, model, input, measurements, mode)
            state(11:13)        = rad2deg(state(11:13));
            estimate(11:13)     = rad2deg(estimate(11:13));
            reference(5:10)     = rad2deg(reference(5:10));
            model(5:7)          = rad2deg(model(5:7));
            measurements(1:6)   = rad2deg(measurements(1:6));
            measurements(26:29) = 60/(2*pi)*measurements(26:29);
            switch lower(mode)
                case 'point',    refColor = [0 0.7 0];
                case 'detumble', refColor = [1 0.5 0];
                case 'off',      refColor = [0.5 0.5 0.5];
                otherwise,       refColor = [0 0.7 0];
            end

            self.time_history(self.index)    = t;
            self.state_history(:,self.index) = state;
            self.estim_history(:,self.index) = estimate;
            self.refer_history(:,self.index) = reference;
            self.model_history(:,self.index) = model;
            self.measurements_history(:,self.index) = measurements;
            self.input_history(:, self.index) = input;
            self.index = self.index + 1;

            if self.plot_translation
                for i = 1:3
                    set(self.handles(i),   'XData', self.time_history, 'YData', self.state_history(i,:));
                    set(self.handles(3+i), 'XData', self.time_history, 'YData', self.state_history(3+i,:));
                end
            end

            if self.plot_orientation
                if self.plot_reference, set(self.handles(25), 'XData', self.time_history, 'YData', self.refer_history(1,:)); end
                if self.plot_model,     set(self.handles(26), 'XData', self.time_history, 'YData', self.model_history(1,:)); end
                if self.plot_estimate,  set(self.handles(27), 'XData', self.time_history, 'YData', self.estim_history(7,:)); end
                set(self.handles(28), 'XData', self.time_history, 'YData', self.state_history(7,:));

                if self.plot_reference, set(self.handles(29), 'XData', self.time_history, 'YData', self.refer_history(2,:)); end
                if self.plot_model,     set(self.handles(30), 'XData', self.time_history, 'YData', self.model_history(2,:)); end
                if self.plot_estimate,  set(self.handles(31), 'XData', self.time_history, 'YData', self.estim_history(8,:)); end
                set(self.handles(32), 'XData', self.time_history, 'YData', self.state_history(8,:));

                if self.plot_reference, set(self.handles(33), 'XData', self.time_history, 'YData', self.refer_history(3,:)); end
                if self.plot_model,     set(self.handles(34), 'XData', self.time_history, 'YData', self.model_history(3,:)); end
                if self.plot_estimate,  set(self.handles(35), 'XData', self.time_history, 'YData', self.estim_history(9,:)); end
                set(self.handles(36), 'XData', self.time_history, 'YData', self.state_history(9,:));

                if self.plot_reference, set(self.handles(37), 'XData', self.time_history, 'YData', self.refer_history(4,:)); end
                if self.plot_model,     set(self.handles(38), 'XData', self.time_history, 'YData', self.model_history(4,:)); end
                if self.plot_estimate,  set(self.handles(39), 'XData', self.time_history, 'YData', self.estim_history(10,:)); end
                set(self.handles(40), 'XData', self.time_history, 'YData', self.state_history(10,:));

                if self.plot_reference, set(self.handles(41), 'XData', self.time_history, 'YData', self.refer_history(5,:)); end
                if self.plot_model,     set(self.handles(42), 'XData', self.time_history, 'YData', self.model_history(5,:)); end
                if self.plot_estimate,  set(self.handles(43), 'XData', self.time_history, 'YData', self.estim_history(11,:)); end
                set(self.handles(44), 'XData', self.time_history, 'YData', self.state_history(11,:));

                if self.plot_reference, set(self.handles(45), 'XData', self.time_history, 'YData', self.refer_history(6,:)); end
                if self.plot_model,     set(self.handles(46), 'XData', self.time_history, 'YData', self.model_history(6,:)); end
                if self.plot_estimate,  set(self.handles(47), 'XData', self.time_history, 'YData', self.estim_history(12,:)); end
                set(self.handles(48), 'XData', self.time_history, 'YData', self.state_history(12,:));

                if self.plot_reference, set(self.handles(49), 'XData', self.time_history, 'YData', self.refer_history(7,:)); end
                if self.plot_model,     set(self.handles(50), 'XData', self.time_history, 'YData', self.model_history(7,:)); end
                if self.plot_estimate,  set(self.handles(51), 'XData', self.time_history, 'YData', self.estim_history(13,:)); end
                set(self.handles(52), 'XData', self.time_history, 'YData', self.state_history(13,:));
            end

            if self.plot_input
                set(self.handles(53), 'XData', self.time_history, 'YData', self.input_history(1,:));
                set(self.handles(54), 'XData', self.time_history, 'YData', self.input_history(2,:));
                set(self.handles(55), 'XData', self.time_history, 'YData', self.input_history(3,:));
                set(self.handles(56), 'XData', self.time_history, 'YData', self.input_history(4,:));
                set(self.handles(57), 'XData', self.time_history, 'YData', self.input_history(5,:));
                set(self.handles(58), 'XData', self.time_history, 'YData', self.input_history(6,:));
                set(self.handles(59), 'XData', self.time_history, 'YData', self.input_history(7,:));
            end

            if self.plot_RPM
                for i = 1:4
                    set(self.handles(59+i), 'XData', self.time_history, ...
                        'YData', self.state_history(13+i,:) * (60/(2*pi)));
                end
            end

            if self.plot_measurements
                for i = 1:3
                    set(self.handles(63+i), 'XData', self.time_history, 'YData', self.measurements_history(i,:));
                end
                for i = 1:3
                    set(self.handles(66+i), 'XData', self.time_history, 'YData', self.measurements_history(3+i,:));
                end
                for i = 1:4
                    set(self.handles(69+i), 'XData', self.time_history, 'YData', self.measurements_history(6+i,:));
                end
                for i = 1:6
                    set(self.handles(73+i), 'XData', self.time_history, 'YData', self.measurements_history(10+i,:));
                end
                for i = 1:3
                    set(self.handles(79+i), 'XData', self.time_history, 'YData', self.measurements_history(16+i,:));
                end
                for i = 1:4
                    set(self.handles(82+i), 'XData', self.time_history, 'YData', self.measurements_history(25+i,:));
                end
            end
            if isempty(self.legend_text_handle) || ~isvalid(self.legend_text_handle)
                self.legend_text_handle = annotation('textbox',[0.75 0.92 0.2 0.05], ...
                    'String',upper(mode), ...
                    'FontWeight','bold','Color',refColor, ...
                    'EdgeColor','none','HorizontalAlignment','right');
            else
                set(self.legend_text_handle, 'String', upper(mode), 'Color', refColor);
            end
        end
    end
end