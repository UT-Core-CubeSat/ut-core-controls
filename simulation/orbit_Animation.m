classdef orbit_Animation < handle
    properties
        % Geometry
        sat_points
        faces
        r_bottom
        r_top
        r_wheel

        % Handles
        ax_orientation
        ax_orbit
        hT                  % hgtransform for entire satellite assembly
        hCube               % 6 patches for cube faces (parent = hT)
        hWheelsSides        % surf handles (parent = hT)
        hWheelsTop
        hWheelsBot
        hOrbitSat           % orbit marker handle
        init_quiver_handles
        ref_quiver_handles

        % Flags
        animate_orientation
        animate_orbit

        % Limits
        xLim_orientation
        yLim_orientation
        zLim_orientation
        xLim_orbit
        yLim_orbit
        zLim_orbit

        % Orbit params
        r_E
        nu_0
        LAN
        i
        omega
        a
        e

        % Palette
        palette_x
        palette_y
        palette_z
    end

    methods
        function self = orbit_Animation(Param)
            % Flags
            self.animate_orientation = Param.animate_orientation;
            self.animate_orbit       = Param.animate_orbit;

            % Limits
            self.xLim_orientation = Param.xLim_orientation;
            self.yLim_orientation = Param.yLim_orientation;
            self.zLim_orientation = Param.zLim_orientation;
            self.xLim_orbit = Param.xLim_orbit;
            self.yLim_orbit = Param.yLim_orbit;
            self.zLim_orbit = Param.zLim_orbit;

            % Orbit params
            self.r_E = Param.r_E;
            self.nu_0 = Param.trueAnomaly;
            self.LAN = Param.RAAN;
            self.i = Param.inclination;
            self.omega = Param.argPeriapsis;
            self.a   = Param.a;
            self.e   = Param.e;

            % Satellite geometry in BODY frame (centered at origin)
            sat_width  = Param.sat_width;
            sat_length = Param.sat_length;
            sat_height = Param.sat_height;

            self.sat_points = [
                sat_width/2,  sat_length/2,  sat_height/2;
                sat_width/2,  sat_length/2, -sat_height/2;
                sat_width/2, -sat_length/2,  sat_height/2;
                sat_width/2, -sat_length/2, -sat_height/2;
                -sat_width/2,  sat_length/2,  sat_height/2;
                -sat_width/2,  sat_length/2, -sat_height/2;
                -sat_width/2, -sat_length/2,  sat_height/2;
                -sat_width/2, -sat_length/2, -sat_height/2];
            self.faces = [
                1 2 4 3;  % +X
                5 6 8 7;  % -X
                1 2 6 5;  % +Y
                3 4 8 7;  % -Y
                1 3 7 5;  % +Z
                2 4 8 6]; % -Z

            self.r_bottom = Param.r_bottom;   % wheel endpoints (BODY)
            self.r_top    = Param.r_top;
            self.r_wheel  = Param.r_wheel;

            self.palette_x = [1 0 0];
            self.palette_y = [0 1 0];
            self.palette_z = [0 0 1];

            % Initial state
            state = Param.states_init(1:13);
            q_init = state(7:10);
            R_init = self.quat2rotm(q_init);
            q_ref  = q_init;
            R_ref  = self.quat2rotm(q_ref);
            % ---------- ORIENTATION FIGURE ----------
            if self.animate_orientation
                fig1 = figure('WindowStyle','docked','Name',"Satellite POV"); clf(fig1);
                self.ax_orientation = axes('Parent', fig1); hold(self.ax_orientation,'on');
                axis(self.ax_orientation,'equal'); grid(self.ax_orientation,'on'); box(self.ax_orientation,'on');
                self.ax_orientation.GridColor = [0.7 0.7 0.7];
                self.ax_orientation.GridAlpha = 0.3;
                self.ax_orientation.LineWidth = 0.5;
                self.ax_orientation.Color = [0.98 0.98 0.99];
                view(self.ax_orientation,[45 25]);
                xlim(self.ax_orientation, self.xLim_orientation);
                ylim(self.ax_orientation, self.yLim_orientation);
                zlim(self.ax_orientation, self.zLim_orientation);

                camlight(self.ax_orientation,'headlight');
                camlight(self.ax_orientation,'right');
                lighting(self.ax_orientation,'gouraud');
                material(self.ax_orientation,'shiny');
                xlabel(self.ax_orientation,"X (m)");
                ylabel(self.ax_orientation,"Y (m)");
                zlabel(self.ax_orientation,"Z (m)");

                % --- Create hgtransform root for all sat geometry ---
                self.hT = hgtransform('Parent', self.ax_orientation);

                % Cube (create faces ONCE, parented to hT)
                face_colors = [0.65, 0.68, 0.72];
                edge_colors = face_colors*0.6;
                V = self.sat_points; % body-frame vertices
                self.hCube = gobjects(6,1);
                for p = 1:6
                    self.hCube(p) = patch('Vertices', V, 'Faces', self.faces(p,:), ...
                        'FaceColor', face_colors, 'FaceAlpha', 0.35, ...
                        'EdgeColor', edge_colors, 'FaceLighting','gouraud', ...
                        'AmbientStrength',0.5,'SpecularStrength',0.1, ...
                        'LineWidth',1.2, 'Parent', self.hT);
                end

                % Reaction wheels (create ONCE in body frame, parent = hT)
                [Xr, Yr, Zr, Xtop, Ytop, Ztop, Xbot, Ybot, Zbot] = self.buildAllWheelsBody();
                wheel_color = [0.15 0.15 0.15];
                nW = size(Xr,3);
                self.hWheelsSides = gobjects(nW,1);
                self.hWheelsTop   = gobjects(nW,1);
                self.hWheelsBot   = gobjects(nW,1);
                for i = 1:nW
                    self.hWheelsSides(i) = surf(self.ax_orientation, Xr(:,:,i), Yr(:,:,i), Zr(:,:,i), ...
                        'FaceColor', wheel_color, 'EdgeColor','none', 'Parent', self.hT);
                    self.hWheelsTop(i) = fill3(self.ax_orientation, Xtop(:,i), Ytop(:,i), Ztop(:,i), ...
                        wheel_color, 'EdgeColor','none', 'Parent', self.hT);
                    self.hWheelsBot(i) = fill3(self.ax_orientation, Xbot(:,i), Ybot(:,i), Zbot(:,i), ...
                        wheel_color, 'EdgeColor','none', 'Parent', self.hT);
                end

                % Initial & Reference frames (quivers)
                scale = sat_length * 3;
                O = [0;0;0];

                colX = self.palette_x; colY = self.palette_y; colZ = self.palette_z;
                self.init_quiver_handles(1) = quiver3(O(1),O(2),O(3), R_init(1,1)*scale, R_init(2,1)*scale, R_init(3,1)*scale, ...
                    '--','Color',colX*0.6,'LineWidth',1.2,'DisplayName','Init X','Parent',self.ax_orientation);
                self.init_quiver_handles(2) = quiver3(O(1),O(2),O(3), R_init(1,2)*scale, R_init(2,2)*scale, R_init(3,2)*scale, ...
                    '--','Color',colY*0.6,'LineWidth',1.2,'HandleVisibility','off','Parent',self.ax_orientation);
                self.init_quiver_handles(3) = quiver3(O(1),O(2),O(3), R_init(1,3)*scale, R_init(2,3)*scale, R_init(3,3)*scale, ...
                    '--','Color',colZ*0.6,'LineWidth',1.2,'HandleVisibility','off','Parent',self.ax_orientation);

                self.ref_quiver_handles(1) = quiver3(O(1),O(2),O(3), R_ref(1,1)*scale, R_ref(2,1)*scale, R_ref(3,1)*scale, ...
                    '-','Color',colX,'LineWidth',2.0,'DisplayName','Ref X','Parent',self.ax_orientation);
                self.ref_quiver_handles(2) = quiver3(O(1),O(2),O(3), R_ref(1,2)*scale, R_ref(2,2)*scale, R_ref(3,2)*scale, ...
                    '-','Color',colY,'LineWidth',2.0,'HandleVisibility','off','Parent',self.ax_orientation);
                self.ref_quiver_handles(3) = quiver3(O(1),O(2),O(3), R_ref(1,3)*scale, R_ref(2,3)*scale, R_ref(3,3)*scale, ...
                    '-','Color',colZ,'LineWidth',2.0,'HandleVisibility','off','Parent',self.ax_orientation);

                legend([self.init_quiver_handles self.ref_quiver_handles], ...
                    {'Init X','Init Y','Init Z','Ref X','Ref Y','Ref Z'}, ...
                    'TextColor',[0.1 0.1 0.1], ...
                    'EdgeColor','k', ...
                    'FontSize',12, ...
                    'Location','northeast', ...
                    'Autoupdate','off');
            end

            % ---------- ORBIT FIGURE ----------
            if self.animate_orbit
                fig2 = figure('WindowStyle','docked','Name',"Orbit"); clf(fig2);
                self.ax_orbit = axes('Parent', fig2); hold(self.ax_orbit,'on');

                % Earth
                [X, Y, Z] = sphere(50);
                surf(self.ax_orbit, X*self.r_E, Y*self.r_E, Z*self.r_E, ...
                    'FaceColor',[0 0 0.5], 'EdgeColor','none');

                % Orbit path
                nus    = linspace(0,2*pi,500);
                r_vals = self.a*(1-self.e^2)./(1+self.e*cos(nus));
                r_PQW  = [r_vals.*cos(nus); r_vals.*sin(nus); zeros(size(nus))];
                Omega  = self.LAN; inc = self.i; w = self.omega;
                RzO = [cos(Omega) -sin(Omega) 0; sin(Omega) cos(Omega) 0; 0 0 1];
                Rxi = [1 0 0; 0 cos(inc) -sin(inc); 0 sin(inc) cos(inc)];
                Rzw = [cos(w) -sin(w) 0; sin(w) cos(w) 0; 0 0 1];
                Q   = RzO*Rxi*Rzw;
                r_ECI= Q*r_PQW;
                plot3(self.ax_orbit, r_ECI(1,:), r_ECI(2,:), r_ECI(3,:), 'r--','LineWidth',1.5);

                % Satellite marker
                r_xyz = state(1:3);
                self.hOrbitSat = plot3(self.ax_orbit, r_xyz(1), r_xyz(2), r_xyz(3), ...
                    'o','MarkerSize',10,'MarkerFaceColor','r','MarkerEdgeColor','k');

                xlabel(self.ax_orbit,"X (m)");
                ylabel(self.ax_orbit,"Y (m)");
                zlabel(self.ax_orbit,"Z (m)");
                axis(self.ax_orbit,'equal'); grid(self.ax_orbit,'on'); view(self.ax_orbit,3);
                light(self.ax_orbit); lighting(self.ax_orbit,'gouraud'); material(self.ax_orbit,'dull');

                xlim(self.ax_orbit,self.xLim_orbit);
                ylim(self.ax_orbit,self.yLim_orbit);
                zlim(self.ax_orbit,self.zLim_orbit);
            end

            % Set initial pose
            self.update(state, q_ref, 'off');
        end

        function update(self, state, reference, mode)
            if ~isempty(self.ax_orientation) && isvalid(self.ax_orientation)
                % Rotate the entire satellite via hgtransform (BODY->ECI)
                q = state(7:10) / norm(state(7:10));
                R = self.quat2rotm(q);
                M = eye(4); M(1:3,1:3) = R; % no translation (body centered)
                if ~isempty(self.hT) && isvalid(self.hT)
                    set(self.hT, 'Matrix', M);
                end

                switch lower(mode)
                    case 'off'
                        for i = 1:length(self.ref_quiver_handles)
                            set(self.ref_quiver_handles(i), 'Visible', 'off');
                        end
                    case 'detumble'
                        for i = 1:length(self.ref_quiver_handles)
                            set(self.ref_quiver_handles(i), 'Visible', 'on', 'Color', [1 0.5 0]);
                        end
                    case 'point'
                        for i = 1:length(self.ref_quiver_handles)
                            colX = self.palette_x; colY = self.palette_y; colZ = self.palette_z;
                            col = [colX; colY; colZ];
                            set(self.ref_quiver_handles(i), 'Visible', 'on', 'Color', col(i,:));
                        end

                        q_ref = reference(1:4);
                        R_ref = self.quat2rotm(q_ref);
                        scale = self.sat_points(1,1)*6;
                        O = [0;0;0];
                        set(self.ref_quiver_handles(1), 'UData', R_ref(1,1)*scale, 'VData', R_ref(2,1)*scale, 'WData', R_ref(3,1)*scale);
                        set(self.ref_quiver_handles(2), 'UData', R_ref(1,2)*scale, 'VData', R_ref(2,2)*scale, 'WData', R_ref(3,2)*scale);
                        set(self.ref_quiver_handles(3), 'UData', R_ref(1,3)*scale, 'VData', R_ref(2,3)*scale, 'WData', R_ref(3,3)*scale);

                    otherwise
                        % Default: hide
                        for i = 1:length(self.ref_quiver_handles)
                            set(self.ref_quiver_handles(i), 'Visible', 'off');
                        end

                end

                if ~isempty(self.ax_orbit) && isvalid(self.ax_orbit)
                    r = state(1:3);
                    if ~isempty(self.hOrbitSat) && isvalid(self.hOrbitSat)
                        set(self.hOrbitSat, 'XData', r(1), 'YData', r(2), 'ZData', r(3));
                    end
                    % keep limits manual to prevent relayout
                    xlim(self.ax_orbit,self.xLim_orbit);
                    ylim(self.ax_orbit,self.yLim_orbit);
                    zlim(self.ax_orbit,self.zLim_orbit);
                end
            end
        end

        % ---------- Helpers ----------
        function R = quat2rotm(~, q)
            q = q / norm(q);
            q0 = q(1); q1 = q(2); q2 = q(3); q3 = q(4);
            R = [1-2*(q2^2+q3^2),   2*(q1*q2 - q0*q3), 2*(q1*q3 + q0*q2);
                2*(q1*q2 + q0*q3), 1-2*(q1^2+q3^2),   2*(q2*q3 - q0*q1);
                2*(q1*q3 - q0*q2), 2*(q2*q3 + q0*q1), 1-2*(q1^2+q2^2)];
        end

        function [Xr, Yr, Zr, Xtop, Ytop, Ztop, Xbot, Ybot, Zbot] = buildAllWheelsBody(self)
            % Build all reaction wheels once in BODY frame; parent later to hT.
            r_top_all    = self.r_top;
            r_bottom_all = self.r_bottom;
            wheel_radius = self.r_wheel;

            n_wheels = size(r_top_all, 2);
            nseg     = 40;             % cylinder resolution
            nring    = nseg + 1;       % points per ring
            [Xc, Yc, Zc] = cylinder(wheel_radius, nseg); % (2 x nring)
            % we'll scale to length L per wheel below

            % Preallocate
            Xr = zeros(2, nring, n_wheels);
            Yr = Xr; Zr = Xr;
            Xtop = zeros(nring, n_wheels);
            Ytop = Xtop; Ztop = Xtop;
            Xbot = Xtop; Ybot = Xtop; Zbot = Xtop;

            for i = 1:n_wheels
                r_top = r_top_all(:,i);
                r_bot = r_bottom_all(:,i);
                v_axis = r_top - r_bot;
                L = norm(v_axis);
                if L < 1e-9, continue; end
                s_hat = v_axis / L;
                mid   = (r_top + r_bot)/2;

                % scale cylinder to actual length centered about origin
                Zloc = (Zc - 0.5) * L;

                % rotate from z-axis to s_hat
                z_axis = [0;0;1];
                if norm(cross(z_axis,s_hat)) < 1e-12
                    R = eye(3);
                else
                    v = cross(z_axis,s_hat);
                    c = dot(z_axis,s_hat);
                    vx = [  0   -v(3)  v(2);
                        v(3)   0   -v(1);
                        -v(2)  v(1)   0 ];
                    R = eye(3) + vx + vx^2 * (1/(1+c));
                end

                % Rotate all points: flatten, multiply, then reshape back
                P = R * [Xc(:)'; Yc(:)'; Zloc(:)'];
                Xtemp = reshape(P(1,:), size(Xc)) + mid(1);
                Ytemp = reshape(P(2,:), size(Yc)) + mid(2);
                Ztemp = reshape(P(3,:), size(Zloc)) + mid(3);

                Xr(:,:,i) = Xtemp;
                Yr(:,:,i) = Ytemp;
                Zr(:,:,i) = Ztemp;

                % End caps
                theta = linspace(0,2*pi,nring);
                circle = wheel_radius * [cos(theta); sin(theta); zeros(size(theta))];
                circle_bottom = R*circle + (mid - 0.5*v_axis);
                circle_top    = R*circle + (mid + 0.5*v_axis);

                Xbot(:,i) = circle_bottom(1,:)';
                Ybot(:,i) = circle_bottom(2,:)';
                Zbot(:,i) = circle_bottom(3,:)';
                Xtop(:,i) = circle_top(1,:)';
                Ytop(:,i) = circle_top(2,:)';
                Ztop(:,i) = circle_top(3,:)';
            end
        end

    end
end