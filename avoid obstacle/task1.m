%RUNMODEL - Main code to run robot model for ENG5009 class
%
%   NOTE: Students should only change the code between *---* markers
%
% Syntax: RunModel
%
% Inputs: none
% Outputs: none
% 
% Other m-files required: 
%   Sensor.m
%   WallGeneration.m
%   DynamicalModel.m
%   DrawRobot.m
%
% Author: Dr. Kevin Worrall
% Last revision: 06-01-2021

%% Preamble
close all;
clear all;
clc;

%% Simulation setup
% Time
simulationTime_total = 150;           % in seconds *------* YOU CAN CHANGE THIS
stepSize_time = 0.05;               % in seconds 

% Initial states and controls
voltage_left  = 6;                  % in volts *------* YOU CAN CHANGE THIS
voltage_right = 6;                  % in volts *------* YOU CAN CHANGE THIS

state_initial = zeros(1,24);        % VARIABLES OF IMPORTANCE:
                                    % state_initial(13): forward velocity,    v, m/s
                                    % state_initial(18): rotational velocity, r, rad/s
                                    % state_initial(19): current x-position,  x, m
                                    % state_initial(20): current y-position,  y, m
                                    % state_initial(24): heading angle,       psi, rad
                                    
% Environment
canvasSize_horizontal = 10;
canvasSize_vertical   = 10;
stepSize_canvas       = 0.01;

% *------------------------------------------*
%  YOU CAN ADD MORE SETUP HERE 
%  (e.g. setup of controller or checkpoints)
    points=[1,1;3,-1;-1,-1;-3,2;0,4]; 
    tolerance = 0.05;
    i = 1;
% *------------------------------------------*

%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
[wall_1, obstacleMatrix] = WallGeneration( -1,  1, 1.2, 1.2, 'h', obstacleMatrix); 
[wall_2, obstacleMatrix] = WallGeneration( -3, -3,  -2,   2, 'v', obstacleMatrix);

% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*


%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;

% Run simulation
for timeStep = 1:timeSteps_total
    % Assign voltage applied to wheels
    voltages = [voltage_left; voltage_left; voltage_right; voltage_right];
    
    % *-------------------------------------*
    %  YOU CAN ADD/CHANGE YOUR CONTROLS HERE
    % *-------------------------------------*


    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(state(timeStep,19:20), points(i,:), tolerance);
    if booleanAtCheckpoint
        i=i+1;
    else
   
        if  newHeadingAngle - state(timeStep,24) > 0.05
            voltage_left = 0.5;
            voltage_right = -0.5;  %turn left to find point
        end
        
        if newHeadingAngle - state(timeStep,24) < -0.05
            voltage_left = -0.5;
            voltage_right = 0.5;  %turn right to find point
        end
                   
        if newHeadingAngle - state(timeStep,24) < 0.05 && newHeadingAngle - state(timeStep,24) > -0.05
            voltage_left = 8;
            voltage_right = 8;  %move to point
        end
    end

    if i > 5
        break
    end

    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    
    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    figure(1); clf; hold on; grid on; axis([-5,5,-5,5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    %plot(wall_1(:,1), wall_1(:,2),'k-');
    %plot(wall_2(:,1), wall_2(:,2),'k-');
    plot(1,1,'*r');
    plot(-1,3,'*r');
    plot(-1,-1,'*r');
    plot(2,-3,'*r');
    plot(4,0,'*r');
    xlabel('y, m'); ylabel('x, m');
end

%% Plot results
% *----------------------------------*
%  YOU CAN ADD OR CHANGE FIGURES HERE
%  don't forget to add axis labels!
% *----------------------------------*

figure(2); hold on; grid on;
title('Robot path(trajectory)');
xlabel('y position(m)'); ylabel('x position,(m)');
xlim([-5.5 5.5]);ylim([-5.5 5.5]);
plot(state(:,20), state(:,19));
plot(1,1,'*r');
plot(-1,3,'*r');
plot(-1,-1,'*r');
plot(2,-3,'*r');
plot(4,0,'*r');
% plot(wall_1(:,1), wall_1(:,2),'k-');
% plot(wall_2(:,1), wall_2(:,2),'k-'); 

figure(3); hold on; grid on;
title('x position that travelled');
xlabel('time(s)'); ylabel('x position(m)');
plot(time, state(:,19));

figure(4); hold on; grid on;
title('heading angle (psi)');
xlabel('time(s)'); ylabel('heading angle(rad)');
plot(time, state(:,24));
