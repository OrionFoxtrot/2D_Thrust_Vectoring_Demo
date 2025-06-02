%% Analysis of Linear System 

% It should be clear, that the linearized system has distinctive
% performance gains that cannot be achieved by the full non linear system.
% As you can test out, the nonlinear sys cannot handle desired positional
% changes of more than a magnitude of aproximately 0.7 (m) while the
% linearized system will be stable for all time. This file attempts to
% provide some insights just on to the linearized systems and the affect of
% adding a observer to the controller. The desired coordinate changes are
% exagerated to show the differences.

clear; clc; close all;

% Controller System
Controlled_System = load("Controller_Sys.mat");
controller_System = Controlled_System.Controlled_System;

% Controller Observer System
Controlled_System = load("observer_Controller_Sys.mat");
controller_Observer_System = Controlled_System.Controlled_System;

t = 0:0.01:7;

desired_X = 20;
desired_Y = 10;
desired_ThetaB = 0;
u = [ones(size(t))*desired_X; ones(size(t))*desired_Y; ones(size(t))*desired_ThetaB]; % 3d input (X, Y, Angle) (desired)

% kSys = controller sys, klSys = controller observer sys
[kSys,t] = lsim(controller_System, u, t, [0;0;0;0;0;0;]); % Pure Controller
[klSys,t] = lsim(controller_Observer_System, u, t, [0;0;0;0;0;0;0;0;0;0;0;0;]); %Observer Controller

l_err = 1;
[klSysError,t] = lsim(controller_Observer_System, u, t, [0;0;0;0;0;0;l_err;l_err;l_err;l_err;l_err;l_err;]); %Observer Controller with Observer Error

ksysX = kSys(:,1);
ksysY = kSys(:,2);
ksysTheta = kSys(:,3);

klsysX = klSys(:,1);
klsysY = klSys(:,2);
klsysTheta = klSys(:,3);

klsysXError = klSysError(:,1);
klsysYError = klSysError(:,2);
klsysThetaError = klSysError(:,3);


subplot(3,1,1);
hold on
title('X coordinate response')
plot(t,ksysX)
plot(t,klsysX, LineStyle="--")
plot(t,klsysXError)
yline(desired_X)
subplot(3,1,2);
hold on
title('Y coordinate response')
plot(t,ksysY)
plot(t,klsysY,LineStyle="--")
plot(t,klsysYError)
yline(desired_Y)
subplot(3,1,3);
yline(desired_ThetaB)
hold on
title('Theta coordinate response')
plot(t,ksysTheta)
plot(t,klsysTheta,LineStyle="--")
plot(t,klsysThetaError)

legend('Controller Response', 'Controller Observer Response', 'Controller Observer w/ Error Response', Location='southeast')
sgtitle('Responses of different system designs')