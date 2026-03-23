function[x,y] = cameracalib(dots,D)
    clf
    
    X = deg2rad(linspace(-20,20,dots)); % up to 20 degrees
    Y = deg2rad(linspace(-20,20,dots));
    [x,y] = meshgrid(X,Y);
    x = D*tan(x);
    y = D*tan(y);

    % Create figure with specific size
    figure(1)
    plot(x,y,'ko','linewidth',6)
    xlabel('inches')
    ylabel('inches')
    axis equal
    % Set paper properties
    set(gcf, 'PaperUnits', 'inches');
    set(gcf, 'PaperSize', [8.5 11]);
    set(gcf, 'PaperPosition', [0 0 8.5 11]);
    
    % Save as PDF
    saveas(gcf, 'grid.png');
end