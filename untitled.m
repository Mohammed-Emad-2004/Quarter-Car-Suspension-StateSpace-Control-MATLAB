% Robots and Robotics Technology Assignment
% Neptun ID: UHCI4H
% Geometric Analysis of a 4-DOF Robotic Arm

clear all; 
clc;

% Constants based on Neptun ID Data
L2 = 366; 
L3 = 146; 
L4 = 112;

% Define 5 test positions (q1, q2, q3, q4)
% Ranges: q1[-90,90], q2[0,50], q3[0,50], q4[-180,180]
test_positions = [
    0,   0,  0,   0;    % Home position
    45,  25, 25,  90;   % Mid-range diagonal
    -90, 50, 50, -180;  % Maximum extension left
    90,  10, 0,   45;   % High vertical, retracted horizontal
    -45, 40, 30,  0     % Arbitrary position
];

fprintf('Kinematic Analysis for Neptun ID: UHCI4H\n');
fprintf('--------------------------------------------------\n');

for i = 1:size(test_positions, 1)
    q1 = deg2rad(test_positions(i,1));
    q2 = test_positions(i,2);
    q3 = test_positions(i,3);
    q4 = deg2rad(test_positions(i,4));
    
    % DH Matrices for each joint
    % Matrix 1: Base to q1 (Rotation)
    T01 = [cos(q1) -sin(q1) 0 0; sin(q1) cos(q1) 0 0; 0 0 1 L2; 0 0 0 1];
    
    % Matrix 2: Vertical movement (Prismatic q2) + 90deg Twist
    T12 = [1 0 0 0; 0 0 -1 0; 0 1 0 q2; 0 0 0 1];
    
    % Matrix 3: Horizontal movement (Prismatic q3)
    T23 = [1 0 0 0; 0 1 0 0; 0 0 1 (L3 + q3); 0 0 0 1];
    
    % Matrix 4: Wrist rotation (q4) and final offset (L4)
    T34 = [cos(q4) -sin(q4) 0 0; sin(q4) cos(q4) 0 0; 0 0 1 L4; 0 0 0 1];
    
    % Total Homogeneous Transformation Matrix (HTM)
    T04 = T01 * T12 * T23 * T34;
    
    % Export Cartesian Coordinates [2]
        x = T04(1,4);
        y = T04(2,4);
        z = T04(3,4);
    
    fprintf('Position %d results:\n', i);
    fprintf('Joints: q1=%.1f, q2=%.1f, q3=%.1f, q4=%.1f\n', ...
            test_positions(i,1), q2, q3, test_positions(i,4));
    fprintf('World Coordinates: X = %.2f, Y = %.2f, Z = %.2f\n', x, y, z);
    fprintf('--------------------------------------------------\n');
end