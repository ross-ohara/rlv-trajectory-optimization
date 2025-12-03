clear
clc
close all


% Model Parameters
m = 100000;
g = 9.81;
L = 50;
r = L/2;
I = 1/12*m*L^2;

% Initial Conditions
x0 = -200;
z0 = 1000;
theta0 = pi/2;
vx0 = 25;
vz0 = -90;
omega0 = 0;
y0 = [x0,z0,theta0,vx0,vz0,omega0]';

% Optimization Parameters
res = 15;
thrustCurve = 0.25*ones(res,1);
deflectionAngle = 0.5*ones(res,1);
tf = 10;
load('res15_initGuess.mat')
Uvect = u_vect_opt;%[thrustCurve;deflectionAngle;tf]; %initial Guess

lb(1:length(Uvect)) = 0;
ub(1:length(Uvect)) = 1;
lb(end) = 1;
ub(end) = 20;

% Optimization Parameters
options = optimoptions('fmincon','Display','iter','Algorithm','sqp',... %interior-point sqp active-set
    'StepTolerance',1e-17,'constraintTolerance',5e-2,'MaxFunctionEvaluations',2e4,'UseParallel',true)

% Trajectory Optimization
[u_vect_opt,fval,exitflag,output,constrviolation] = fmincon(@(Uvect) CostFCN(m,g,r,I,Uvect,res,y0),Uvect,[],[],[],[],lb,ub,...
    @(Uvect) nonlcon(m,g,r,I,Uvect,res,y0),options);

%% Evaluate Solution
% Initiailize Simulation
tf = u_vect_opt(end);
tspan = [0 tf];

% Run Simulation
[t,X] = ode45(@(t,x) SS_NLDE(t,x,m,g,r,I,u_vect_opt,res), tspan, y0);

% Resample Optimal Control Solution
thrustCurve = u_vect_opt(1:res);
deflectionAngle =  u_vect_opt(res+1:2*res);
tf = u_vect_opt(end);
tvect = linspace(0,tf,res);
resampledTC = interp1(tvect,thrustCurve,t,'pchip');
resampledDA = interp1(tvect,deflectionAngle,t,'pchip');

% Visualize Fligth Data
figure(1)
subplot(2,3,1)
plot(t,X(:,1:2))
legend('X_E','Z_E')
xlabel('Time (s)')
ylabel('Position (m)')
grid on
grid minor

subplot(2,3,2)
plot(t,X(:,4:5))
legend('X_E','Z_E')
xlabel('Time (s)')
ylabel('Velocity (m/s)')
grid on
grid minor

subplot(2,3,3)
plot(t,X(:,3))
legend('\theta')
xlabel('Time (s)')
ylabel('Orientation (rad)')
grid on
grid minor

subplot(2,3,4)
plot(X(:,1),X(:,2),'k')
legend('Flight Path')
xlabel('X (m)')
ylabel('Y (m)')
grid on
grid minor
axis equal

subplot(2,3,5)
Ft = resampledTC*15; %15 meganewtons
plot(t,Ft)
legend('Thrust')
xlabel('Time (s)')
ylabel('Force (MN)')
grid on
grid minor

subplot(2,3,6)
theta_t = 2*(resampledDA-0.5)*pi/8; %
plot(t,theta_t)
legend('\delta')
xlabel('Time (s)')
ylabel('Gimbal Angle (rad)')
grid on
grid minor

%% Build Gif from Flight Data

% Initiailize Simulation
tf = u_vect_opt(end);
nframe = round(30*tf);
tspan = linspace(0,tf,nframe);

% Run Simulation
[t,X] = ode45(@(t,x) SS_NLDE(t,x,m,g,r,I,u_vect_opt,res), tspan, y0);

% Resample Optimal Control Solution
thrustCurve = u_vect_opt(1:res);
deflectionAngle =  u_vect_opt(res+1:2*res);
tf = u_vect_opt(end);
tvect = linspace(0,tf,res);
resampledTC = interp1(tvect,thrustCurve,t,'pchip');
resampledDA = interp1(tvect,deflectionAngle,t,'pchip');

figure(2)
vid = VideoWriter('newfile.mp4','MPEG-4');
vid.FrameRate = nframe/tf;
open(vid)
for i = 1:length(t)

clf
plot(X(end,1),X(end,2),'or')
hold on

% Draw Body
rCOM = [X(i,1),X(i,2)+50/2]';
theta = X(i,3);
Rot = [cos(theta),sin(theta);-sin(theta),cos(theta)];
p1 = rCOM+Rot*[0;65/2];
p2 = rCOM+Rot*[9/2;50/2];
p3 = rCOM+Rot*[9/2;-50/2];
p4 = rCOM+Rot*[-9/2;-50/2];
p5 = rCOM+Rot*[-9/2;50/2];

% Draw Propulsion
thetat = (resampledDA(i)-0.5)*2*pi/8;
RotT = [cos(thetat),-sin(thetat),;sin(thetat),cos(thetat)];
t1 = rCOM+Rot*[0;-50/2];
t2 = t1 + Rot*RotT*[0;-50*resampledTC(i)]; %*resampledTC(i)

plot(rCOM(1),rCOM(2),'og')
plot([p1(1),p2(1)],[p1(2),p2(2)],'k')
plot([p2(1),p3(1)],[p2(2),p3(2)],'k')
plot([p3(1),p4(1)],[p3(2),p4(2)],'k')
plot([p4(1),p5(1)],[p4(2),p5(2)],'k')
plot([p5(1),p1(1)],[p5(2),p1(2)],'k')
plot([t1(1),t2(1)],[t1(2),t2(2)],'r')

pad = 100;
xlim([rCOM(1)-pad,rCOM(1)+pad])
ylim([rCOM(2)-pad,rCOM(2)+pad])
grid on
grid minor
xlabel('X (m)')
ylabel('Z (m)')
%axis padded equal

frame = getframe(gcf);
writeVideo(vid,frame)

end
for i = 1:30
frame = getframe(gcf);
writeVideo(vid,frame)
end

close(vid)






% Differential Equation for Starship Non-Linear Differential Equations
function xdot = SS_NLDE(t,x,m,g,r,I,Uvect,res)

% Parse Optimization Variable
thrustCurve = Uvect(1:res);
deflectionAngle =  Uvect(res+1:2*res);
tf = Uvect(end);

tvect = linspace(0,tf,res);
Ut = interp1(tvect,thrustCurve,t,'pchip');
Uda = interp1(tvect,deflectionAngle,t,'pchip');

Ft = Ut*15*10^6; %15 meganewtons
theta_t = 2*(Uda-0.5)*pi/8; %

% Sum of Forces and Moments
Fx = Ft*sin(theta_t)*cos(x(3))+Ft*cos(theta_t)*sin(x(3));
Fz = -Ft*sin(theta_t)*sin(x(3))+Ft*cos(theta_t)*cos(x(3))-m*g;
M = Ft*sin(theta_t)*r;

% Assign Xdot Vector
Vx = x(4);
Vz = x(5);
omega = x(6);
ax = Fx/m;
az = Fz/m;
alpha = M/I;

xdot = [Vx,Vz,omega,ax,az,alpha]';
end

% Cost function
function J = CostFCN(m,g,r,I,Uvect,res,y0)
% Initiailize Simulation
tf = Uvect(end);
tspan = [0 tf];

% Run Simulation
[t,X] = ode45(@(t,x) SS_NLDE(t,x,m,g,r,I,Uvect,res), tspan, y0);

thrustCurve = Uvect(1:res);
deflectionAngle =  Uvect(res+1:2*res);
tvect = linspace(0,tf,res);
TC = interp1(tvect,thrustCurve,t,'pchip');
VC = interp1(tvect,deflectionAngle,t,'pchip');

J =  trapz(t,TC.*TC) + trapz(t,VC.*VC);% + sum(X(end,:).*X(end,:));
end

% Constraints boundary conditions
function [c,ceq] = nonlcon(m,g,r,I,Uvect,res,y0)

% Initiailize Simulation
tspan = [0 Uvect(end)];

% Run Simulation
[t,X] = ode45(@(t,x) SS_NLDE(t,x,m,g,r,I,Uvect,res), tspan, y0);

xdot = SS_NLDE(t(end),X(end,:),m,g,r,I,Uvect,res);

ceq = [X(end,1);...
    X(end,2);...
    X(end,3);...
    X(end,4);...
    X(end,5);...
    X(end,6);...
    xdot(4);... 
    xdot(5);...
    xdot(6);...
    ];

c = [max(abs(X(:,3)))-pi/2;...
    -min(X(:,2))];
end

