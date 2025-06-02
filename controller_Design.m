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

save('Controlled_Sys','Controlled_System','gains')
