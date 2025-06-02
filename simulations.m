clear; clc; close all;

Controlled_System = load("Controlled_Sys.mat");
Controlled_System = Controlled_System.Controlled_System;

t = 0:0.01:7;

desired_X = 20;
desired_Y = 10;
desired_ThetaB = 0;
u = [ones(size(t))*desired_X; ones(size(t))*desired_Y; ones(size(t))*desired_ThetaB]; % 3d input (X, Y, Angle) (desired)

[yv,t] = lsim(Controlled_System, u, t, [0;0;0;0;0;0;]);


x = yv(:,1);
y = yv(:,2);
thetab = yv(:,3);
W = 3; %width m
L = 1; % length m
animateThrustVectorBox(x,y,thetab,L,W)

%% non linear sim version
clear; clc; close all;

W = 3; %width m
L = 1; % length m
m = 20; %kg
g = 9.81; %ms-1
Ig = 1/12*m*(W^2+L^2); %MOI

Controlled_System = load("Controlled_Sys.mat");
K = Controlled_System.gains;

x_d      = 0.5;     
y_d      = 0.5;    
thetab_d = deg2rad(0);    

X_d = [ x_d; y_d; thetab_d; 0; 0; 0 ];

ff_term = [0 m*g]; % technically this should be the equilibrium point
% but its aproximately m*g, so lets leave it at that. 

t = 0:0.1:10;
fdynamic    = @(t,X) non_lin_2d_sim(t,X,K, W,L,m,g,Ig, X_d, ff_term);
[t yv] = ode45(fdynamic,t,[0;0;0;0;0;0;]);

x = yv(:,1);
y = yv(:,2);
thetab = yv(:,3);
animateThrustVectorBox(x,y,thetab,L,W)
%% Non Linear Simulations
function Xdot = non_lin_2d_sim(t,X, gains, W,L,m,g,Ig, X_d, ff_term)


    x = X(1);
    y = X(2);
    thetab = X(3);
    x_dot = X(4); %unused --
    y_dot = X(5);
    thetab_dot = X(6); 
    

    % 
    e = X - X_d;
    tau = ff_term - gains*e;
   
    thetat = tau(1);
    F = tau(2);
    
    
    x_ddot = -F/m * sin(thetat);
    y_ddot = F* cos(thetat)/m - g;
    thetab_ddot = (F*L)/(2*Ig) * (sin(thetab)*cos(thetat)-cos(thetab)*sin(thetat));
    
    
    Xdot = [x_dot y_dot thetab_dot x_ddot y_ddot thetab_ddot]';
end


%%
function animateThrustVectorBox(x, y, thetab, L, W)
% animateThrustVectorBox Animate a 2D rigid rectangular box + thrust arrow
    N = numel(x);
    % Precompute half‐dimensions:
    hL = L/2;
    hW = W/2;

    % Rectangle in its own body‐frame, in CCW order:
    %
    %   ( +L/2, +W/2 ) front/right
    %   ( −L/2, +W/2 ) front/left
    %   ( −L/2, −W/2 ) back/left
    %   ( +L/2, −W/2 ) back/right

    corners_body = [ ...
         +hL,  -hL,  -hL,  +hL;    % x‐coordinates of the 4 corners
         +hW,  +hW,  -hW,  -hW ];  % y‐coordinates of the 4 corners

 
    figure;
    axis equal;
    hold on;
    grid on;
    xlabel('x');
    ylabel('y');
    title('Thrust‐Vectoring Box Animation');

    margin = max(L,W)*1.5;
    xmin = min(x) - margin;
    xmax = max(x) + margin;
    ymin = min(y) - margin;
    ymax = max(y) + margin;
    axis([xmin, xmax, ymin, ymax]);

    R0 = [ cos(thetab(1)), -sin(thetab(1));
           sin(thetab(1)),  +cos(thetab(1)) ];
    corners_rot0 = R0 * corners_body;
    X0 = corners_rot0(1,:) + x(1);
    Y0 = corners_rot0(2,:) + y(1);
    hBox = patch( X0, Y0, 'b', 'FaceAlpha', 0.3 ); 

    
    thrust_length = L*0.75;  % scale arrow to 75% of box length
    thrust_dir0 = R0 * [0; 1];
    hArrow = quiver( x(1), y(1), ...
                     thrust_length*thrust_dir0(1), ...
                     thrust_length*thrust_dir0(2), ...
                     'r', 'LineWidth', 2, 'MaxHeadSize', 2 );

    for k = 1:N
 
        Rk = [ cos(thetab(k)), -sin(thetab(k));
               sin(thetab(k)),  +cos(thetab(k)) ];

        corners_rot = Rk * corners_body;
        Xc = corners_rot(1,:) + x(k);
        Yc = corners_rot(2,:) + y(k);

        set(hBox, 'XData', Xc, 'YData', Yc);

      
        thrust_dir = Rk * [0; 1];
        set(hArrow, ...
            'XData',  x(k), ...
            'YData',  y(k), ...
            'UData',  thrust_length * thrust_dir(1), ...
            'VData',  thrust_length * thrust_dir(2) );

        drawnow;
        pause(0.01);
    end

end


function animateThrustVectorBoxSave(x, y, thetab, L, W)

    N = numel(x);
    hL = L/2;
    hW = W/2;

    corners_body = [ ...
         +hL,  -hL,  -hL,  +hL;   
         +hW,  +hW,  -hW,  -hW ];

    fig = figure;
    axis equal;
    hold on;
    grid on;
    xlabel('x');
    ylabel('y');
    title('Thrust‐Vectoring Box Animation');

    margin = max(L,W)*1.5;
    xmin = min(x) - margin;
    xmax = max(x) + margin;
    ymin = min(y) - margin;
    ymax = max(y) + margin;
    axis([xmin, xmax, ymin, ymax]);


    R0 = [ cos(thetab(1)), -sin(thetab(1));
           sin(thetab(1)),  +cos(thetab(1)) ];
    corners_rot0 = R0 * corners_body;
    X0 = corners_rot0(1,:) + x(1);
    Y0 = corners_rot0(2,:) + y(1);
    hBox = patch( X0, Y0, 'b', 'FaceAlpha', 0.3 );

    thrust_length = L*0.75; 
    thrust_dir0 = R0 * [0; 1];
    hArrow = quiver( x(1), y(1), ...
                     thrust_length*thrust_dir0(1), ...
                     thrust_length*thrust_dir0(2), ...
                     'r', 'LineWidth', 2, 'MaxHeadSize', 2 );

    v = VideoWriter('animation.mp4', 'MPEG-4');
    v.FrameRate = 30;  
    open(v);

  
    for k = 1:N
       
        Rk = [ cos(thetab(k)), -sin(thetab(k));
               sin(thetab(k)),  +cos(thetab(k)) ];
        corners_rot = Rk * corners_body;
        Xc = corners_rot(1,:) + x(k);
        Yc = corners_rot(2,:) + y(k);
        set(hBox, 'XData', Xc, 'YData', Yc);


        thrust_dir = Rk * [0; 1];
        set(hArrow, ...
            'XData',  x(k), ...
            'YData',  y(k), ...
            'UData',  thrust_length * thrust_dir(1), ...
            'VData',  thrust_length * thrust_dir(2) );

        drawnow;

        frame = getframe(fig);
        writeVideo(v, frame);

        pause(0.01);
    end

    close(v);
    close(fig);
end
