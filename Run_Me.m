% ==============================================================================
% MATLAB Source Codes for "Spatio-temporal decomposition: a knowledge-based
% initialization strategy for parallel parking motion optimization". 

% ==============================================================================

%   Copyright (C) 2016 Bai Li
%   Useers must cite the following article when utilizing these source codes to produce new contributions. 
%   Bai Li et al., "Spatio-temporal decomposition: a knowledge-based initialization
%   strategy for parallel parking motion optimization", Knowledge-Based Systems 2016.
%   http://dx.doi.org/10.1016/j.knosys.2016.06.008

% ==============================================================================

% If there are inquiries, feel free to contact libai@zju.edu.cn or directly,
% libaioutstanding@163.com

% ==============================================================================

close all
clear all
clc

addpath Supporting_Files
% Fundamental parametric settings
global PPP0
load OBS_info.mat

% Fundamental User-Specified Parametric Settings

NE = 20; % Number of sub-periods in orthogonal collocation direct transcription method

SL = 6; % Parking slot length
SW  = 2.5; % Parking slot width
CL = 4; % Lane width

x0 = -3; % State x at t = 0
y0 = 1.5; % State y at t = 0
theta0 = 0; % State theta at t = 0
v0 = 0; % State v at t = 0

amax = 0.75; % Bound of acceleration
vmax = 1.8; % Bound of velocity
dcurmax = 1.2; % Bound of steering angular velocity
phymax = 0.576; % Bound of steering angle
n = 0.96; % Front overhang length
l = 2.8; % Distance between the front and back wheel axes
m = 0.929; % Rear overhang length
b = 0.971; % Vehicle half width

tf = 9999; % A large enough tf for Spatio-Temporal Decomposition strategy
flag = 0; % A flag reflects whether at least one successful optimization process is achieved
% during the iterative pre-solving process using Spatio-Temporal Decomposition strategy

record = zeros(1,(NE-1)); % A variable tracks the updates of tf (opt. objective)

for ii = 1:(NE-1)
    
    if (flag == 1) % When flag = 1, previous successful optimization is accomplished, then we can use the initial guess.
        [tf0,result] = Opti_Park(NE,SL,SW,CL,x0,y0,theta0,v0,amax,vmax,dcurmax,phymax,n,l,m,b,ii,[0,1]); % Details of Opti_Park can be found using "help" function.
    else % When flag = 0, there is no available initial guess.
        [tf0,result] = Opti_Park(NE,SL,SW,CL,x0,y0,theta0,v0,amax,vmax,dcurmax,phymax,n,l,m,b,ii,[0,0]);
    end
    
    if (result == 1) % Result = 1 indicates that previous optimization process is successful.
        tf = tf0; % Update tf.
        Init_Generation('SZJ.INIVAL'); % Generate initial guess and store in SZJ.INIVAL
        flag = 1; % Given that previous optimization process is successful, flag is set to 1 and no longer 0.
    end
    
    record(ii) = tf; % Record current active tf.
  
end

plot(record)

pause(5)


% Formal Parking Motion Optimization Process without Spatio-temporal decomposition
% Note that the pre-solving process is influencing the formal optimization
% process by the active initial guess.

Opti_Park(NE,SL,SW,CL,x0,y0,theta0,v0,amax,vmax,dcurmax,phymax,n,l,m,b,ii,[1,1]);


draw_figure % Draw the parking motions.
draw_profile % Draw optimized profiles.
pause(30)

play_dynamic_result % Play dynamic parking process.