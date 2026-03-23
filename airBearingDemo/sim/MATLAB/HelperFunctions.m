classdef HelperFunctions < handle
    methods

        function q = quatFromTwoVectors(~, v_from, v_to)
            % quatFromTwoVectors  Quaternion rotating v_from to v_to
            %
            %   q = quatFromTwoVectors(v_from, v_to)
            %
            % Inputs:
            %   v_from  - 3x1 initial vector
            %   v_to    - 3x1 target vector
            %
            % Output:
            %   q       - 4x1 unit quaternion [q0; qx; qy; qz] (scalar first)
            %
            % This function computes the minimal rotation quaternion that rotates
            % vector v_from to align with v_to.

            % Normalize input vectors
            v1 = v_from / (norm(v_from) + 1e-12);
            v2 = v_to   / (norm(v_to)   + 1e-12);

            % Compute dot and cross
            dotProd = dot(v1, v2);
            crossProd = cross(v1, v2);

            % Handle edge cases
            if dotProd < -0.999999
                % Opposite vectors — choose arbitrary perpendicular axis
                orth = [1;0;0];
                if abs(v1(1)) > 0.9
                    orth = [0;1;0];
                end
                rotAxis = cross(v1, orth);
                rotAxis = rotAxis / norm(rotAxis + 1e-12);
                q = [0; rotAxis]; % 180° rotation
            else
                q = [1 + dotProd; crossProd];
                q = q / norm(q + 1e-12);
            end
        end


        function JD = julianDate(~,dateTime)
            y = year(dateTime);
            m = month(dateTime);
            d = day(dateTime);
            h = hour(dateTime);
            mi = minute(dateTime);
            s = second(dateTime);
            if m <= 2
                y = y - 1;
                m = m + 12;
            end
            A = floor(y / 100);
            B = 2 - A + floor(A / 4);
            JD = floor(365.25*(y + 4716)) + floor(30.6001*(m + 1)) + d + B - 1524.5 ...
                + (h + mi/60 + s/3600)/24;
        end

        function q = quatMultiply(~, q1, q2)
            w1 = q1(1); v1 = q1(2:4);
            w2 = q2(1); v2 = q2(2:4);
            w = w1*w2 - dot(v1,v2);
            v = w1*v2 + w2*v1 + cross(v1,v2);
            q = [w; v];
        end

        function C = dcmeci2ecef(~, JD)
            GMST = mod(280.46061837 + 360.98564736629*(JD - 2451545), 360);
            theta = deg2rad(GMST);
            C = [cos(theta) sin(theta) 0;
                -sin(theta) cos(theta) 0;
                0 0 1];
        end

        function ecef = eci2ecef(self, eci, JD)
            C = self.dcmeci2ecef(JD);
            ecef = C * eci;
        end

        function qc = quatconj(~, q)
            qc = [q(1); -q(2:4)];
        end

        function v_rot = quatrotate(self, q, v)
            qv = [0; v(:)];
            v_rot = self.quatMultiply(self.quatMultiply(q, qv), self.quatconj(q));
            v_rot = v_rot(2:4);
        end

        function [lat, lon, alt] = ecef2lla(~, r)
            x = r(1); y = r(2); z = r(3);
            % WGS-84 ellipsoid (Earth)
            a  = 6378137.0;                 % semi-major axis [m]
            f  = 1/298.257223563;
            e2 = f*(2-f);                   % first eccentricity squared
            lon = atan2(y, x);
            rho = sqrt(x^2 + y^2);
            lat = atan2(z, rho*(1 - e2));
            lat_prev = 0;
            while abs(lat - lat_prev) > 1e-12
                N = a / sqrt(1 - e2*sin(lat)^2);
                alt = rho / cos(lat) - N;
                lat_prev = lat;
                lat = atan2(z, rho*(1 - e2^2*N/(N + alt)));
            end
            N = a / sqrt(1 - e2*sin(lat)^2);
            alt = rho / cos(lat) - N;
        end

        function C = dcmecef2ned(~, lat, lon)
            sL = sin(lat); cL = cos(lat);
            s_lambda = sin(lon); c_lambda = cos(lon);
            C = [-sL*c_lambda, -sL*s_lambda, cL;
                -s_lambda,     c_lambda,     0;
                -cL*c_lambda, -cL*s_lambda, -sL];
        end

        function [B_NED, B_total] = wrldmagm(self, lat, lon, alt, decYear)
            % WRLDMAGM  Computes Earth's magnetic field in NED frame (WMM)
            %
            % Inputs:
            %   lat, lon  - geodetic latitude [rad], longitude [rad]
            %   alt       - altitude above ellipsoid [m]
            %   decYear   - decimal year (e.g. 2025.35)
            %
            % Outputs:
            %   B_NED     - 3x1 magnetic field vector [T] (North, East, Down)
            %   B_total   - total field magnitude [T]

            WMM = self.loadWMMcoeffs('world_mag_model.txt');
            a = WMM.a;                % reference radius
            f = 1/298.257223563;      % WGS-84 flattening
            e2 = f*(2-f);

            % === Convert geodetic to geocentric coordinates ===
            N = a / sqrt(1 - e2*sin(lat)^2);
            X = (N + alt) * cos(lat);
            Z = (N*(1 - e2) + alt) * sin(lat);
            r = sqrt(X^2 + Z^2);
            phi_gc = atan((1 - e2)*tan(lat));  % geocentric latitude
            theta = pi/2 - phi_gc;             % colatitude

            % === Precompute cos(m*lon), sin(m*lon) ===
            Nmax = size(WMM.g,1) - 1;
            cosmlon = cos((0:Nmax)*lon);
            sinmlon = sin((0:Nmax)*lon);

            % === Time adjustment of coefficients ===
            dt = decYear - WMM.epoch;
            g = 1e-9 * (WMM.g + WMM.dg*dt);    % [T]
            h = 1e-9 * (WMM.h + WMM.dh*dt);    % [T]

            % === Compute Schmidt semi-normalized P_n^m(cosθ) ===
            P = zeros(Nmax+1, Nmax+1);
            P(1,1) = 1; % P_0^0

            for n = 1:Nmax
                P(n+1,n+1) = (2*n - 1) * sin(theta) * P(n,n);
                P(n+1,n)   = (2*n - 1) * cos(theta) * P(n,n-1+1);
                for m = 0:n-2
                    P(n+1,m+1) = ((2*n - 1)*cos(theta)*P(n,m+1) ...
                        - (n + m - 1)*P(n-1,m+1)) / (n - m);
                end
            end
            

            % Schmidt normalization
            S = zeros(Nmax+1, Nmax+1);
            for n = 0:Nmax
                for m = 0:n
                    if m == 0
                        S(n+1,m+1) = sqrt(2*n + 1);
                    else
                        S(n+1,m+1) = sqrt(2 * (2*n + 1) * factorial(n - m) / factorial(n + m));
                    end
                end
            end
            P = P .* S;

            % === Compute field components in spherical coords ===
            Br = 0; Bt = 0; Bp = 0;
            a_over_r = a / r;

            for n = 1:Nmax
                ar_pwr = a_over_r^(n+2);
                for m = 0:n
                    i = n + 1;
                    j = m + 1;

                    V = g(i,j)*cosmlon(j) + h(i,j)*sinmlon(j);
                    W = -g(i,j)*sinmlon(j) + h(i,j)*cosmlon(j);

                    dP = self.diff_legendre(P, n, m, theta);

                    Br = Br - (n+1)*ar_pwr * V * P(i,j);
                    Bt = Bt - ar_pwr * V * dP;
                    if abs(sin(theta)) > 1e-10
                        Bp = Bp - ar_pwr * (m*P(i,j)/sin(theta)) * W;
                    else
                        Bp = 0;
                    end
                end
            end

            % === Convert spherical to NED ===
            B_N = -Bt;
            B_E =  Bp;
            B_D = -Br;
            B_NED = [B_N; B_E; B_D];
            B_total = norm(B_NED);
        end

        function dP = diff_legendre(~, P, n, m, theta)
            if n == 0
                dP = 0;
                return;
            end
            dP = (n * cos(theta) * P(n+1, m+1) - (n + m) * P(n, m+1)) / sin(theta);
        end



        function WMM = loadWMMcoeffs(~, filename)
            % Read the coefficient file
            fid = fopen(filename);
            if fid < 0, error('Could not open file %s', filename); end

            % Read coefficients
            data = textscan(fid, '%f %f %f %f %f %f');
            fclose(fid);

            % Unpack columns
            n = data{1}; m = data{2};
            g = data{3}; h = data{4};
            dg = data{5}; dh = data{6};

            % Determine maximum degree
            N = max(n);

            % Allocate arrays
            WMM.g  = zeros(N+1);
            WMM.h  = zeros(N+1);
            WMM.dg = zeros(N+1);
            WMM.dh = zeros(N+1);

            % Fill arrays
            for i = 1:length(n)
                WMM.g(n(i)+1, m(i)+1)  = g(i);
                WMM.h(n(i)+1, m(i)+1)  = h(i);
                WMM.dg(n(i)+1, m(i)+1) = dg(i);
                WMM.dh(n(i)+1, m(i)+1) = dh(i);
            end

            WMM.epoch = 2025.0;  % Extract from file header manually if needed
            WMM.a = 6371200;     % WMM reference radius (m)
        end

        function R = earth2sun(~,jd)
            % get the position of the sun in the ECI frame given juliandate
            % INPUTS
            % jd - Juliandate
            % OUTPUTS
            % R - normalized vector pointing from earth to sun
            %--------------------------------------------------------------

            % calculate time from Julian Date
            T = (jd - 2451545.0) / 36525;

            % calculate geometric mean longitude and mean anomaly of the sun
            L_0 = 280.46646 + 36000.76983*T + 0.0003032*T^2;
            M   = 357.52911 + 35999.05029*T - 0.0001537*T^2;

            % eccentricity of the earths orbit
            e_earth = 0.016708634 - 0.000042037*T - 0.0000001267*T^2;

            % suns equation of center
            C = (1.914602 - 0.004817*T - 0.000014*T^2)*sind(M) +...
                (0.019993 - 0.000101*T)*sind(2*M)+...
                0.000289*sind(3*M);

            % find true longitude and true anomaly of the sun
            true_long = L_0 + C;
            true_anom =   M + C;

            % distance from earth center to sun center in AU
            numer = interp1([2000;2100],[0.9997218;0.9997232],2025);
            D = numer*(1-e_earth^2) / (1 + e_earth*cosd(true_anom));

            % calculate obliquity
            a_p = 23 + 26/60 + 21.448/3600;
            b = 46.8150/3600;
            c = 0.00059/3600;
            d = 0.001813/3600;

            epsilon = a_p - b*T - c*T^2 + d*T^3;

            % calculate declination and right ascension
            alpha = atan2d(cosd(epsilon)*sind(true_long), cosd(true_long));
            delta = asind(sind(epsilon)*sind(true_long));

            % calculate rectangular coordinates
            x = D*cosd(delta)*cosd(alpha);
            y = D*cosd(delta)*sind(alpha);
            z = D*sind(delta);

            % normalize and pack position vector
            R = [x;y;z]/norm([x;y;z]);
        end

        
    end
end