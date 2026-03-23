clc; clear; close all;

% Define paper size in inches (Letter size: 8.5 x 11 inches)
paper_width = 8.5;
paper_height = 11;

% Define dot spacing in inches
dot_spacing = 1.0;

% Generate grid points
[x, y] = meshgrid(0.5:dot_spacing:paper_width, 0.5:dot_spacing:paper_height);

% Create figure
fig = figure;
hold on;
scatter(x(:), y(:), 10, 'k', 'filled'); % Small black dots
hold off;

% Formatting
xlim([0, paper_width]);
ylim([0, paper_height]);
axis off;
set(gca, 'Position', [0 0 1 1]); % Remove margins

% Save as PDF
set(fig, 'PaperUnits', 'inches', 'PaperSize', [paper_width paper_height], ...
         'PaperPosition', [0 0 paper_width paper_height]);
print(fig, 'star_tracker_grid.pdf', '-dpdf', '-r300');

disp('PDF saved as star_tracker_grid.pdf');
