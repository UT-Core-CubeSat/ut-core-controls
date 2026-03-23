function plotQuaternionAxes()
    quaternions = load("gains.mat").p(:,1:4);

    % plotQuaternionAxes - Plot unit vectors representing rotation axes from quaternions
    % 
    % Input:
    %   quaternions - Nx4 matrix where each row is a quaternion [w x y z]
    %                 w is the scalar part, [x y z] is the vector part
    
    % Check input
    if size(quaternions, 2) ~= 4
        error('Input must be an Nx4 matrix where each row is a quaternion [w x y z]');
    end
    
    % Create figure
    figure;
    hold on;
    
    % Set up the axes
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Quaternion Rotation Axes');
    
    % Draw coordinate axes for reference
    axisLength = 1.5;
    line([0, axisLength], [0, 0], [0, 0], 'Color', 'r', 'LineWidth', 2);
    line([0, 0], [0, axisLength], [0, 0], 'Color', 'g', 'LineWidth', 2);
    line([0, 0], [0, 0], [0, axisLength], 'Color', 'b', 'LineWidth', 2);
    
    % Extract rotation axes from quaternions and plot them
    numQuaternions = size(quaternions, 1);
    colors = jet(numQuaternions); % Generate different colors for each vector
    
    for i = 1:numQuaternions
        q = quaternions(i, :);
        
        % Extract the rotation axis
        % For a unit quaternion q = [cos(θ/2), sin(θ/2)*axis]
        % The vector part (normalized) gives us the rotation axis
        axis_vector = q(2:4);
        
        % Normalize to get unit vector
        % If the vector part is close to zero, the rotation is minimal
        norm_axis = norm(axis_vector);
        %if norm_axis < 1e-6
            % Almost no rotation, skip this quaternion
         %   continue;
        %end
        
        % Normalize the axis
        %axis_vector = axis_vector / norm_axis;
        
        % Plot the unit vector from origin
        quiver3(0, 0, 0, axis_vector(1), axis_vector(2), axis_vector(3), 1, ...
            'Color', colors(i,:), 'LineWidth', 1.5, 'MaxHeadSize', 0.2);
    end
    
    % Add a legend with the number of vectors plotted
    legend(['Plotted ' num2str(numQuaternions) ' quaternion axes']);
    
    % Set viewing angle
    view(45, 30);
    
    hold off;
end

