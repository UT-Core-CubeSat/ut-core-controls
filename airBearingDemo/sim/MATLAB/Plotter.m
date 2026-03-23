classdef Plotter < handle
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
        function self = Plotter(Param)
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
            self.state_history        = NaN(11, N);
            self.estim_history        = NaN(11, N);
            self.refer_history        = NaN(10, N);
            self.model_history        = NaN(7, N);
            self.input_history        = NaN(7,  N);
            self.measurements_history = NaN(13, N);
            self.index = 1;

            self.buildFigures(Param);
        end

        function buildFigures(self, Param)

            t = self.time_history;
            rgb = {'r','g','b'};

            
            %% Orientation
            if self.plot_orientation
                figure('Windowstyle','docked','name','Orientation');
                clf;
                sgtitle('Orientation')

                subplot(7,1,1); hold on; grid on;
                if self.plot_reference; self.handles(1) = plot(self.time_history, self.refer_history(1,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(2) = plot(self.time_history, self.model_history(1,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(3) = plot(self.time_history, self.estim_history(1,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(4) = plot(self.time_history, self.state_history(1,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_0$','Interpreter','latex')

                subplot(7,1,2); hold on; grid on;
                if self.plot_reference; self.handles(5) = plot(self.time_history, self.refer_history(2,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(6) = plot(self.time_history, self.model_history(2,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(7) = plot(self.time_history, self.estim_history(2,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(8) = plot(self.time_history, self.state_history(2,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_1$',"Interpreter","latex")

                subplot(7,1,3); hold on; grid on;
                if self.plot_reference; self.handles(9) = plot(self.time_history, self.refer_history(3,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(10) = plot(self.time_history, self.model_history(3,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(11) = plot(self.time_history, self.estim_history(3,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(12) = plot(self.time_history, self.state_history(3,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_2$',"Interpreter","latex")

                subplot(7,1,4); hold on; grid on;
                if self.plot_reference; self.handles(13) = plot(self.time_history, self.refer_history(4,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(14) = plot(self.time_history, self.model_history(4,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(15) = plot(self.time_history, self.estim_history(4,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(16) = plot(self.time_history, self.state_history(4,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$q_3$',"Interpreter","latex")

                subplot(7,1,5); hold on; grid on;
                if self.plot_reference; self.handles(17) = plot(self.time_history, self.refer_history(5,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(18) = plot(self.time_history, self.model_history(5,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(19) = plot(self.time_history, self.estim_history(5,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(20) = plot(self.time_history, self.state_history(5,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_x$ (deg/s)','Interpreter','latex')

                subplot(7,1,6); hold on; grid on;
                if self.plot_reference; self.handles(21) = plot(self.time_history, self.refer_history(6,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(22) = plot(self.time_history, self.model_history(6,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(23) = plot(self.time_history, self.estim_history(6,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(24) = plot(self.time_history, self.state_history(6,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_y$ (deg/s)','Interpreter','latex')

                ax = subplot(7,1,7); hold on; grid on;
                if self.plot_reference; self.handles(25) = plot(self.time_history, self.refer_history(7,:), '-g', "DisplayName", "Reference",'LineWidth',Param.line_width); end
                if self.plot_model;     self.handles(26) = plot(self.time_history, self.model_history(7,:), '-b', "DisplayName", "Model",    'LineWidth',Param.line_width); end
                if self.plot_estimate;  self.handles(27) = plot(self.time_history, self.estim_history(7,:), '-r', "DisplayName", "Estimate", 'LineWidth',Param.line_width); end
                self.handles(28) = plot(self.time_history, self.state_history(7,:), '-k', "DisplayName", "State", 'LineWidth',Param.line_width);
                ylabel('$\omega_z$ (deg/s)','Interpreter','latex')
                xlabel("t - time (s)");

                % legend for top row (reference/model/estimate/state)
                legend_lines = self.handles(25:28);
                legend_handle = legend(legend_lines([self.plot_reference,self.plot_model,self.plot_estimate,true]),"Orientation","horizontal","FontSize",10);
                legend_handle.Position(1:2) = ax.Position(1:2)+[0.15,-0.1];
            end

            %% Inputs
            if self.plot_input
                % Wheels
                figure('Windowstyle','docked','name','Inputs (RW)');
                clf; sgtitle('Inputs')
                subplot(4,1,1); hold on; grid on;
                self.handles(29) = plot(self.time_history, self.input_history(1,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_1$ (Nm)','Interpreter','latex')
                subplot(4,1,2); hold on; grid on;
                self.handles(30) = plot(self.time_history, self.input_history(2,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_2$ (Nm)','Interpreter','latex')
                subplot(4,1,3); hold on; grid on;
                self.handles(31) = plot(self.time_history, self.input_history(3,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_3$ (Nm)','Interpreter','latex')
                subplot(4,1,4); hold on; grid on;
                self.handles(32) = plot(self.time_history, self.input_history(4,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_4$ (Nm)','Interpreter','latex'); xlabel("t - time (s)")

                % MTQs
                figure('Windowstyle','docked','name','Inputs (MTQ)');
                clf; sgtitle('Inputs')
                subplot(3,1,1); hold on; grid on;
                self.handles(33) = plot(self.time_history, self.input_history(5,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_1$ (Nm)','Interpreter','latex')
                subplot(3,1,2); hold on; grid on;
                self.handles(34) = plot(self.time_history, self.input_history(6,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_2$ (Nm)','Interpreter','latex')
                subplot(3,1,3); hold on; grid on;
                self.handles(35) = plot(self.time_history, self.input_history(7,:), '-k','LineWidth',Param.line_width); ylabel('$\tau_3$ (Nm)','Interpreter','latex'); xlabel("t - time (s)")
            end

            %% Reaction wheel RPMs
            if self.plot_RPM
                figure('Windowstyle','docked','name','RW Speeds');
                clf; hold on; grid on;
                sgtitle('Reaction Wheel Speeds');
                colors = {'r','g','b','k'};
                for i = 1:4
                    % handles 60..63 map to states 14..17
                    self.handles(35+i) = plot(self.time_history, ...
                        self.state_history(7+i,:) * (60/(2*pi)), ...
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

                % Accelerometers (1:3) -> handles 64..66
                subplot(2,3,1); hold on; grid on;
                for i = 1:3
                    self.handles(39+i) = plot(t, self.measurements_history(i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Accelerometers'); ylabel("$m/s^2$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                % Gyros (4:6) -> handles 67..69
                subplot(2,3,2); hold on; grid on;
                for i = 1:3
                    self.handles(42+i) = plot(t, self.measurements_history(3+i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Gyros'); ylabel("$rad/s$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                % Magnetometers (17:19) -> handles 80..82
                subplot(2,3,5); hold on; grid on;
                for i = 1:3
                    self.handles(45+i) = plot(t, self.measurements_history(6+i,:), rgb{i}, 'LineWidth',Param.line_width);
                end
                title('Magnetometers'); ylabel("$T$","Interpreter","latex"); xlabel("t (s)"); legend({'x','y','z'},'Location','best');

                % Wheel Tachometers (26:29) -> handles 83..86
                subplot(2,3,6); hold on; grid on;
                wtc = {'r','g','b','k'};
                for i = 1:4
                    self.handles(48+i) = plot(t, self.measurements_history(9+i,:), wtc{i}, 'LineWidth',Param.line_width);
                end
                title('Wheel Tachometers'); ylabel("$RPM$","Interpreter","latex"); xlabel("t (s)"); legend({'Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4'},'Location','best')
            end
            
        end

        function update(self, t, state, estimate, reference, model, input, measurements, mode)
            % Unit conversions
            state(5:7)        = rad2deg(state(5:7));
            estimate(5:7)     = rad2deg(estimate(5:7));
            reference(5:10)     = rad2deg(reference(5:10));
            model(5:7)          = rad2deg(model(5:7));
            measurements(4:6)   = rad2deg(measurements(4:6));     % (kept as in your original)
            measurements(10:13) = 60/(2*pi)*measurements(10:13);
            switch lower(mode)
                case 'point',    refColor = [0 0.7 0];
                case 'detumble', refColor = [1 0.5 0];
                case 'off',      refColor = [0.5 0.5 0.5];
                otherwise,       refColor = [0 0.7 0];
            end

            % Save into histories
            self.time_history(self.index)    = t;
            self.state_history(:,self.index) = state;
            self.estim_history(:,self.index) = estimate;
            self.refer_history(:,self.index) = reference;
            self.model_history(:,self.index) = model;
            self.measurements_history(:,self.index) = measurements;
            self.input_history(:, self.index) = input;
            self.index = self.index + 1;

            % === Update plots ===

            % Orientation (unchanged)
            if self.plot_orientation
                if self.plot_reference, set(self.handles(1), 'XData', self.time_history, 'YData', self.refer_history(1,:)); end
                if self.plot_model,     set(self.handles(2), 'XData', self.time_history, 'YData', self.model_history(1,:)); end
                if self.plot_estimate,  set(self.handles(3), 'XData', self.time_history, 'YData', self.estim_history(1,:)); end
                set(self.handles(4), 'XData', self.time_history, 'YData', self.state_history(1,:));

                if self.plot_reference, set(self.handles(5), 'XData', self.time_history, 'YData', self.refer_history(2,:)); end
                if self.plot_model,     set(self.handles(6), 'XData', self.time_history, 'YData', self.model_history(2,:)); end
                if self.plot_estimate,  set(self.handles(7), 'XData', self.time_history, 'YData', self.estim_history(2,:)); end
                set(self.handles(8), 'XData', self.time_history, 'YData', self.state_history(2,:));

                if self.plot_reference, set(self.handles(9), 'XData', self.time_history, 'YData', self.refer_history(3,:)); end
                if self.plot_model,     set(self.handles(10), 'XData', self.time_history, 'YData', self.model_history(3,:)); end
                if self.plot_estimate,  set(self.handles(11), 'XData', self.time_history, 'YData', self.estim_history(3,:)); end
                set(self.handles(12), 'XData', self.time_history, 'YData', self.state_history(3,:));

                if self.plot_reference, set(self.handles(13), 'XData', self.time_history, 'YData', self.refer_history(4,:)); end
                if self.plot_model,     set(self.handles(14), 'XData', self.time_history, 'YData', self.model_history(4,:)); end
                if self.plot_estimate,  set(self.handles(15), 'XData', self.time_history, 'YData', self.estim_history(4,:)); end
                set(self.handles(16), 'XData', self.time_history, 'YData', self.state_history(4,:));

                if self.plot_reference, set(self.handles(17), 'XData', self.time_history, 'YData', self.refer_history(5,:)); end
                if self.plot_model,     set(self.handles(18), 'XData', self.time_history, 'YData', self.model_history(5,:)); end
                if self.plot_estimate,  set(self.handles(19), 'XData', self.time_history, 'YData', self.estim_history(5,:)); end
                set(self.handles(20), 'XData', self.time_history, 'YData', self.state_history(5,:));

                if self.plot_reference, set(self.handles(21), 'XData', self.time_history, 'YData', self.refer_history(6,:)); end
                if self.plot_model,     set(self.handles(22), 'XData', self.time_history, 'YData', self.model_history(6,:)); end
                if self.plot_estimate,  set(self.handles(23), 'XData', self.time_history, 'YData', self.estim_history(6,:)); end
                set(self.handles(24), 'XData', self.time_history, 'YData', self.state_history(6,:));

                if self.plot_reference, set(self.handles(25), 'XData', self.time_history, 'YData', self.refer_history(7,:)); end
                if self.plot_model,     set(self.handles(26), 'XData', self.time_history, 'YData', self.model_history(7,:)); end
                if self.plot_estimate,  set(self.handles(27), 'XData', self.time_history, 'YData', self.estim_history(7,:)); end
                set(self.handles(28), 'XData', self.time_history, 'YData', self.state_history(7,:));
            end

            % Inputs (optional)
            if self.plot_input
                set(self.handles(29), 'XData', self.time_history, 'YData', self.input_history(1,:));
                set(self.handles(30), 'XData', self.time_history, 'YData', self.input_history(2,:));
                set(self.handles(31), 'XData', self.time_history, 'YData', self.input_history(3,:));
                set(self.handles(32), 'XData', self.time_history, 'YData', self.input_history(4,:));
                set(self.handles(33), 'XData', self.time_history, 'YData', self.input_history(5,:));
                set(self.handles(34), 'XData', self.time_history, 'YData', self.input_history(6,:));
                set(self.handles(35), 'XData', self.time_history, 'YData', self.input_history(7,:));
            end

            % Reaction wheel RPMs (combined: handles 60..63, states 14..17)
            if self.plot_RPM
                for i = 1:4
                    set(self.handles(35+i), 'XData', self.time_history, ...
                        'YData', self.state_history(7+i,:) * (60/(2*pi)));
                end
            end

            % Measurements (multi-line groups)
            if self.plot_measurements
                % Accelerometers 64..66 : rows 1..3
                for i = 1:3
                    set(self.handles(39+i), 'XData', self.time_history, 'YData', self.measurements_history(i,:));
                end
                % Gyros 67..69 : rows 4..6
                for i = 1:3
                    set(self.handles(42+i), 'XData', self.time_history, 'YData', self.measurements_history(3+i,:));
                end
                % Magnetometers 80..82 : rows 17..19
                for i = 1:3
                    set(self.handles(45+i), 'XData', self.time_history, 'YData', self.measurements_history(6+i,:));
                end
                % Wheel tach 83..86 : rows 20..23
                for i = 1:4
                    set(self.handles(48+i), 'XData', self.time_history, 'YData', self.measurements_history(9+i,:));
                end
            end
            % === Mode legend text ===
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
