%% Switching Control Controller Synthesis for Autonomous Vehicle Collision Avoidance
close all
clear all
clc

% Initialize values
% States x,y,phi
x_new = zeros(10000,1);
y_new = zeros(10000,1);
xf = 0;
yf = 0;
xp = 0;
yp = 0;
xt = 100; % target points
yt = 100;
d1=sqrt((xt^2)+(yt^2)); % Distance to target
phin = 0;
phi1 = 0;
phip = 0;
setphi = 0;

% Input v,w
U = [0; 0];
lastw = 0;

% Error
e1 = 0;
l = 0;
e2 = 0;
eo1 = 0;
eo2 = 0;
E = [e1; e2];

% Controller
k1 = 10;
K = 0.1; 
K0 = 0.1;
kp = .7;
ki = 0;

% Counters
m=0;
n=0;
Tnew = zeros(10000,1) ;
m= m+1 ;
DelT = 0.1 ;
c = 5 ;
d = 5 ;

while(n<10000)
    n=n+1; %counter for loop
    tau = 100; %safety distance
    %setting up position of object x in [25,50]; y in [25,50]
    for x0= 25:.1:75
        for y0 = 50:.1:75
            d = sqrt((x0-x_new(n,1))*(x0-x_new(n,1)) + (y0-y_new(n,1))*(y0-y_new(n,1))); 
            if d<tau
                tau=d;
                xf=x0;
                yf=y0;
            end
        end
    end
 % Calculating error values for control law
    E(1)= xt - x_new(n,1);
    E(2) = yt - y_new(n,1);
    E0(1) = xf - x_new(n,1);
    E0(2) = yf -y_new(n,1);
 % Switching control law depending on object distance
    if tau>7
        v = K*sqrt( E(1)^2 + E(2)^2) ;
        w = kp*(atan2(E(2),E(1))- phin) ;
    elseif tau<=7
        v = K0*sqrt( E0(1)^2 + E0(2)^2) ;
        w = -1*kp*(atan2(E0(2),E0(1))- phin) ;
    end
    %If statement to determine if vehicle should keep moving (max of 10 m/s) or stop
    % Must most solve because kinematic model not stable for fast speeds
    if (v>0)
        v = min(10,v) ;
    else
        v = 0 ;
    end
        % Updating position of vehicle
    Tnew(n+1,1) = DelT*n ;
    phin = w*DelT + phip ;
    phip= phin ;
    xndot = (v*cos(phin));
    yndot = (v*sin(phin));
    x_new(n+1,1) = xndot*DelT + xp ;
    y_new(n+1,1) = yndot*DelT + yp ;
    xp = x_new(n+1,1) ;
    yp = y_new(n+1,1) ;
end

figure
hold on; grid on;
plot(x_new,y_new)
ylim([0,100]);
title('Trajectory of vehicle when encountering object')
xlabel('object at x[25,75] and y[50,75], tau = 7')