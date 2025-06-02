%% Pure Controller Design
clear; clc; close all;

matricies = load("state_space_matricies.mat");
A = matricies.A;
B = matricies.B;
C = matricies.C;

gains = place(A,B, [-10 -11 -12 -13 -1+i -1-i] );

% New A Matrix
newA = A-B*gains;

new_state = ss(newA, B, C, 0);

% calculating DC gain
tf_new = tf(new_state);
dc_gain = evalfr(tf_new,0);
% dc_gain = 1./dc_gain
dc_gain = pinv(dc_gain);


%Final TF
Controlled_System = ss(newA, B*dc_gain, C, 0);

save('controller_Sys','Controlled_System','gains')


%% Controller Observer Design:
clear; clc; close all;

matricies = load("state_space_matricies.mat");
A = matricies.A;
B = matricies.B;
C = matricies.C;

poles_desired_K = [-10 -11 -12 -13 -1+i -1-i];
K = place(A,B, poles_desired_K );

poles_desired_L = poles_desired_K*3;
L = place(A', C', poles_desired_L)';

At = [ A-B*K             B*K
       zeros(size(A))    A-L*C ];

Bt = [    B
       zeros(size(B)) ];

Ct = [ C    zeros(size(C)) ];

new_state = ss(At, Bt, Ct, 0);

% calculating DC gain
tf_new = tf(new_state);
dc_gain = evalfr(tf_new,0);
% dc_gain = 1./dc_gain
dc_gain = pinv(dc_gain);

%Final TF
Controlled_System = ss(At, Bt*dc_gain, Ct, 0);

save('observer_Controller_Sys','Controlled_System','K','L')