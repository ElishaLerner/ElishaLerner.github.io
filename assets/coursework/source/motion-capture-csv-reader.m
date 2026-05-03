format long

dataFile = fullfile(fileparts(mfilename('fullpath')), 'ml-position-data.csv');
T = readtable(dataFile);

inputs = T(:, {'D1', 'D2', 'H1', 'Motordist'});
positions = T(:, {'X', 'Y', 'Z'});

disp('Input conditions used for the adaptive-origami position predictor:')
disp(unique(inputs))

disp('Mean measured point-of-interest position for each input condition:')
summary = groupsummary(T, {'D1', 'D2', 'H1', 'Motordist'}, 'mean', {'X', 'Y', 'Z'});
disp(summary)

figure
scatter3(positions.X, positions.Y, positions.Z, 40, T.Motordist, 'filled')
grid on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
title('Measured adaptive-origami endpoint positions')
colorbar
