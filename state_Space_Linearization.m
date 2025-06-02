clear; clc; close all;
syms x y thetab 
syms x_dot y_dot thetab_dot

syms thetat F % inputs

% syms m g Ig W L % constants
W = 3; %width m
L = 1; % length m
m = 20; %kg
g = 9.81; %ms-1
Ig = 1/12*m*(W^2+L^2); %MOI

L = [x_dot;
    y_dot;
    thetab_dot;
    -F/m * sin(thetat);
    F* cos(thetat)/m - g;
    (F*L)/(2*Ig) * (sin(thetab)*cos(thetat)-cos(thetab)*sin(thetat))
    ];


z = [diff(L,x) diff(L,y) diff(L,thetab) diff(L, x_dot) diff(L, y_dot) diff(L, thetab_dot) ]; % A
A = subs(z, [x y thetab x_dot y_dot thetab_dot thetat F], [0 0 0 0 0 0 0 196.2]);

u = [diff(L, thetat) diff(L,F)];
B = subs(u, [x y thetab x_dot y_dot thetab_dot thetat F], [0 0 0 0 0 0 0 196.2]);

C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     ]; % only observing x, y, thetab


wc = [B A*B A^2*B A^3*B A^4*B A^5*B];
if(rank(wc) == height(wc))
    disp('sys is reachable')
end

wo = [C; C*A; C*A^2; C*A^3; C*A^4; C*A^5];
if(rank(wo) == width(wo))
    disp('sys is observable')
end


A = double(A);
B = double(B);
C = double(C);
save('state_space_matricies','A', 'B', 'C')



%% eqilibrium points

clear; clc; close all;
syms x y thetab 
syms x_dot y_dot thetab_dot

syms thetat F % inputs

% syms m g Ig W L % constants
W = 1; %width m
L = 3; % length m
m = 20; %kg
g = 9.81; %ms-1
Ig = 1/12*m*(W^2+L^2); %MOI

f1 = 0 == -F/m * sin(thetat);
f2 = 0 == F*cos(thetat)/m - g;
f3 = 0 == (F*L)/(2*Ig) * (sin(thetab)*cos(thetat)-cos(thetab)*sin(thetat));

sol = vpasolve([f1 f2 f3], [F thetab thetat]);

sol.F
sol.thetab
sol.thetat