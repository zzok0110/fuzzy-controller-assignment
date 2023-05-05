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
simulationTime_total = 450;           % in seconds *------* YOU CAN CHANGE THIS
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
canvasSize_horizontal = 10.01;
canvasSize_vertical   = 10.01;
stepSize_canvas       = 0.01;

% *------------------------------------------*
%  YOU CAN ADD MORE SETUP HERE 
%  (e.g. setup of controller or checkpoints)
    points=[1.5,4.5;-1,2;-4,-1;-1,-4;4,0;0,0]; 
    tolerance = 0.05;
    i = 1;
% *------------------------------------------*

%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls
% --> the variable "obstacleMatrix" is updated for each added wall
% outside walls
[wall_1, obstacleMatrix] = WallGeneration( -5, 5, 5, 5, 'h', obstacleMatrix); 
[wall_2, obstacleMatrix] = WallGeneration( -5, -5,  -5,   5, 'h', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( -5, -5, -5, 5, 'v', obstacleMatrix); 
[wall_4, obstacleMatrix] = WallGeneration( 5, 5, -5, 5, 'v', obstacleMatrix);

% *---------------------------*
%  YOU CAN ADD MORE WALLS HERE
% *---------------------------*

% Obstacles
[wall_5, obstacleMatrix] = WallGeneration( -2.5, 2.5, 2.5, 2.5, 'h', obstacleMatrix); 
[wall_6, obstacleMatrix] = WallGeneration( -3, -3, -3, 0.5, 'v', obstacleMatrix); 
[wall_7, obstacleMatrix] = WallGeneration( 1, 1, 0, 0, 'h', obstacleMatrix); 
[wall_8, obstacleMatrix] = WallGeneration( 1, 1, -2, 0, 'v', obstacleMatrix); 

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


    fz = readfis('fn');
    sensorOut = Sensor(state(timeStep,19), state(timeStep,20),state(timeStep,24), obstacleMatrix);
    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(state(timeStep,19:20), points(i,:), tolerance);
    

    if booleanAtCheckpoint
            i=i+1;
    end

    if i > 6
        break
    end
    
    if i == 1||i == 2||i==3||i==4
        angle = newHeadingAngle - state(timeStep,24);
        final(1)=sensorOut(1);
        final(2)=sensorOut(2);
        final(3)=angle;
        vlr = evalfis(fz,final);
        voltage_left = vlr([fz.Outputs.Name]=='motor left');
        voltage_right = vlr([fz.Outputs.Name]=='motor right');
    end

    zz = readfis('fnsecond');

    if i==5
        angle = newHeadingAngle - state(timeStep,24);
        final(1)=sensorOut(1);
        final(2)=sensorOut(2);
        final(3)=angle;
        vlr = evalfis(zz,final);
        voltage_left = vlr([zz.Outputs.Name]=='motor left');
        voltage_right = vlr([zz.Outputs.Name]=='motor right');
    end

    kk = readfis('last');
    if i==6
        angle = wrapToPi(newHeadingAngle - state(timeStep,24));
        final(1)=sensorOut(1);
        final(2)=sensorOut(2);
        final(3)=angle;
        vlr = evalfis(kk,final);
        voltage_left = vlr([kk.Outputs.Name]=='motor left');
        voltage_right = vlr([kk.Outputs.Name]=='motor right');
    end

    
    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;
    
    % Plot robot on canvas  *------* YOU CAN ADD STUFF HERE
    figure(1); clf; hold on; grid on; axis([-5.5,5.5,-5.5,5.5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
    plot(wall_1(:,1), wall_1(:,2),'k-');
    plot(wall_2(:,1), wall_2(:,2),'k-'); 
    plot(wall_3(:,1), wall_3(:,2),'k-'); 
    plot(wall_4(:,1), wall_4(:,2),'k-'); 
    plot(wall_5(:,1), wall_5(:,2),'k-'); 
    plot(wall_6(:,1), wall_6(:,2),'k-'); 
    plot(wall_7(:,1), wall_7(:,2),'k-'); 
    plot(wall_8(:,1), wall_8(:,2),'k-'); 
    plot(4.5,1.5,'*r');
    plot(2,-1,'*r');
    plot(-1,-4,'*r');
    plot(-4,-1,'*r');
    plot(0,4,'*r');
    plot(0,0,'*r');
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
plot(4.5,1.5,'*r');
plot(2,-1,'*r');
plot(-1,-4,'*r');
plot(-4,-1,'*r');
plot(0,4,'*r');
plot(0,0,'*r');
plot(wall_1(:,1), wall_1(:,2),'k-');
plot(wall_2(:,1), wall_2(:,2),'k-'); 
plot(wall_3(:,1), wall_3(:,2),'k-'); 
plot(wall_4(:,1), wall_4(:,2),'k-'); 
plot(wall_5(:,1), wall_5(:,2),'k-'); 
plot(wall_6(:,1), wall_6(:,2),'k-'); 
plot(wall_7(:,1), wall_7(:,2),'k-'); 
plot(wall_8(:,1), wall_8(:,2),'k-'); 


figure(3); hold on; grid on;
title('x position that travelled');
xlabel('time(s)'); ylabel('x position(m)');
plot(time, state(:,19));

figure(4); hold on; grid on;
title('heading angle (psi)');
xlabel('time(s)'); ylabel('heading angle(rad)');
plot(time, state(:,24));
