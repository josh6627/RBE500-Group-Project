clear;
clc;
close all;

% Location of the csv file
filename = fullfile(getenv("HOME"), "Robotics", "rbe500", "group-project", "joint_position_log.csv");

% Import the CSV data.
data = readtable(filename);

time = data.time_s;
referencePosition = data.reference_position_m;
currentPosition = data.current_position_m;

% Plot reference and actual joint positions.
figure;
plot(time, referencePosition, "--", "LineWidth", 1.8);

hold on;

plot(time, currentPosition, "LineWidth", 1.8);

xlabel("Time (s)");
ylabel("Joint Position (m)");
title("Prismatic Joint Position Response");

legend("Reference Position", "Current Position", "Location", "best");

grid on;
hold off;