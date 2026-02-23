%% Simulation Equivalence Checker
% Compares C++ output (simulation_data.csv) vs MATLAB output (matlab_data.csv)
% and calculates the percentage of values that match within a tolerance.

clear; clc;

% =========================================================================
% CONFIGURATION
% =========================================================================
file_cpp    = 'simulation_data.csv'; % Make sure this is in the folder
file_matlab = 'matlab_data.csv';     % Generate this using MATLABCSV.m

% Tolerance for "Match"
% 1e-6 is standard for float vs double comparisons.
% If logic is IDENTICAL, you might get 1e-12.
tolerance = 1e-6; 

% =========================================================================
% 1. LOAD DATA
% =========================================================================
if ~isfile(file_cpp) || ~isfile(file_matlab)
    error('Files not found! Ensure both csv files exist in the current folder.');
end

opts = detectImportOptions(file_cpp);
opts.VariableNamingRule = 'preserve';

fprintf('Loading C++ Data... ');
tab_cpp = readtable(file_cpp, opts);
data_cpp = table2array(tab_cpp);
fprintf('Done.\n');

fprintf('Loading MATLAB Data... ');
tab_matlab = readtable(file_matlab, opts);
data_matlab = table2array(tab_matlab);
fprintf('Done.\n');

% =========================================================================
% 2. ALIGN DATA
% =========================================================================
% If one simulation ran slightly longer or logged one extra step, crop to the minimum.
n_rows = min(size(data_cpp, 1), size(data_matlab, 1));
n_cols = min(size(data_cpp, 2), size(data_matlab, 2));

if size(data_cpp, 2) ~= size(data_matlab, 2)
    warning('Column count mismatch! C++: %d, MATLAB: %d. Comparing first %d columns.', ...
        size(data_cpp, 2), size(data_matlab, 2), n_cols);
end

A = data_cpp(1:n_rows, 1:n_cols);
B = data_matlab(1:n_rows, 1:n_cols);

% =========================================================================
% 3. COMPARE
% =========================================================================
% Calculate Absolute Difference
diff_matrix = abs(A - B);

% Handle NaN comparisons (NaN != NaN, so we manually fix them)
nan_mask = isnan(A) & isnan(B);
diff_matrix(nan_mask) = 0; % If both are NaN, difference is 0.

% Check against tolerance
pass_mask = diff_matrix <= tolerance;

% Overall Statistics
total_elements = numel(pass_mask);
passed_elements = sum(pass_mask, 'all');
percent_match = (passed_elements / total_elements) * 100;

% Find worst error
[max_diff, idx] = max(diff_matrix, [], 'all', 'linear');
[r_worst, c_worst] = ind2sub(size(diff_matrix), idx);
col_name_worst = tab_cpp.Properties.VariableNames{c_worst};

% =========================================================================
% 4. REPORT
% =========================================================================
fprintf('\n======================================================\n');
fprintf(' EQUIVALENCE REPORT\n');
fprintf('======================================================\n');
fprintf('Files Compared    : %d rows, %d cols\n', n_rows, n_cols);
fprintf('Tolerance Used    : %g\n', tolerance);
fprintf('OVERALL MATCH     : %.4f %%\n', percent_match);
fprintf('Worst Discrepancy : %g (at Row %d, Col %s)\n', max_diff, r_worst, col_name_worst);

fprintf('\n--- Breakdown by Subsystem ---\n');
fprintf('(Tolerance = %g)\n\n', tolerance);

% Helper to check range of columns
check_range = @(name, cols) fprintf('%-15s : %.2f %%\n', name, ...
    (sum(pass_mask(:, cols), 'all') / numel(pass_mask(:, cols))) * 100);

% Column Indices (Based on main.cpp / MATLABCSV.m)
% 1      : t
% 2-18   : True States
% 19-35  : Estimates
% 36-45  : Reference
% 46-52  : Inputs
% 53-81  : Measurements
% 82-88  : Model States
% 89     : Mode

check_range('Time', 1);
check_range('True States', 2:18);
check_range('Estimates', 19:35);
check_range('Reference', 36:45);
check_range('Inputs (Tau)', 46:52);
check_range('Measurements', 53:81);
check_range('Model States', 82:88);
check_range('Mode Logic', 89);

fprintf('======================================================\n');

% Optional: Plot the difference of the worst column
if percent_match < 100
    figure;
    plot(A(:,1), diff_matrix(:, c_worst));
    title(['Discrepancy in ' col_name_worst]);
    xlabel('Time (s)');
    ylabel('Abs Error');
    grid on;
end