function plotEvenlySpacedSpherePoints(numPoints)
    % plotEvenlySpacedSpherePoints - Generate and plot evenly distributed points on a unit sphere
    %
    % Input:
    %   numPoints - Approximate number of points to distribute (default: 90)
    
    % Set default number of points if not provided
    if nargin < 1
        numPoints = 90;
    end
    
    % Generate evenly distributed points using the Fibonacci sphere method
    % This is one of the best methods for approximately uniform distribution
    points = fibonacciSphere(numPoints);
    
    % Create figure
    figure;
    hold on;
    
    % Plot the points
    scatter3(points(:,1), points(:,2), points(:,3), 50, 'filled', 'MarkerFaceColor', 'b');
    
    % Optional: Plot the wireframe of the unit sphere for reference
    [X,Y,Z] = sphere(30);
    surf(X, Y, Z, 'FaceAlpha', 0.1, 'EdgeAlpha', 0.2, 'FaceColor', 'none');
    
    % Set up the axes
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(sprintf('Approximately %d Evenly Distributed Points on Unit Sphere', numPoints));
    
    % Add coordinate axes
    axisLength = 1.2;
    line([0, axisLength], [0, 0], [0, 0], 'Color', 'r', 'LineWidth', 2);
    line([0, 0], [0, axisLength], [0, 0], 'Color', 'g', 'LineWidth', 2);
    line([0, 0], [0, 0], [0, axisLength], 'Color', 'b', 'LineWidth', 2);
    
    % Set viewing angle
    view(45, 30);
    
    % Display the number of points
    legend(['Number of points: ' num2str(size(points, 1))]);
    
    hold off;
end



% Example usage: Create and plot approximately 90 points
% plotEvenlySpacedSpherePoints(90);