function gains = calculate_gains(n)
    gains = [];

    axis_vectors = fibonacciSphere(n);

    for i = 1:n
        axis_vector = axis_vectors(i,:);

        real_values = [0, 0.25, 0.5, 0.75, 1];

        for j = 1:length(real_values)
            quat = [real_values(j);axis_vector'];

            quat = quat / norm(quat);

            gain = calculate_gain(quat);

            gains(end + 1,:) = gain;
        end
    end
end

function points = fibonacciSphere(n)
    % Generate points using the Fibonacci sphere algorithm
    % This creates almost evenly distributed points on a unit sphere
    
    points = zeros(n, 3);
    
    % Golden angle in radians
    phi = (1 + sqrt(5)) / 2;
    
    for i = 0:n-1
        % Calculate the spherical coordinates
        y = 1 - (i / (n-1)) * 2;  % y goes from 1 to -1
        radius = sqrt(1 - y*y);   % radius at y
        
        theta = 2 * pi * i / phi; % golden angle increment
        
        % Convert to Cartesian coordinates
        x = cos(theta) * radius;
        z = sin(theta) * radius;
        
        points(i+1,:) = [x, y, z];
    end
end

function gain = calculate_gain(q)
    Parameters

    [A,B,C,D] = modrod_state_space([q;zeros(3,1)], zeros(3,1), parameters.I);
    [K_x,~,~] = lqr(A,B,parameters.Q, parameters.R);
    %[K_x,~,~] = place(A,B,[-1,-1,-1,-2,-2,-2]);
    n_x=size(A,1);
    n_y=size(C,1);
    n_u=size(B,2);
    %Non-Zero Set Point
    QPM = [A,B;C,D];
    P = QPM^-1;
    P12= P(1:n_x,end-n_y+1:end);
    P22 = P(end-n_u+1,end-n_y+1:end);
    K_r = [K_x * P12 + P22];

   gain = [q', reshape(K_x,1,[]), reshape(K_r,1,[])] ;
end