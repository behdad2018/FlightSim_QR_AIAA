% This is the main run page of the quad-copter simulation
% written by Behdad Davoudi, Computational Aeroscience Lab 
% Aerospace Engineering Department, University of Michigan
% 2018

clearvars -except U
clc
dtr = pi/180;

%% Environment parameters
g = 9.81;   % gravitational acceleration (m/s^2)
% drag coefficients
Cd = 0.001; % was not used eventually
% new drag implementation (used)
cbar=0.04;
%% LES Wind this need to be done one time if you cleared the workspace already
%load('./wind/U.mat')
global U
%% Dryden model horizontal wind
% sample noise time appears has to be set as one of the parameters of the
% Dryden wind model. If the step size of the solver is changed, it is
% cruicial to update it!
% we use a predefined mean wind speed
load('wind_table.mat'); 
% wind magnitude will be set through a interpolation block in:
%Wind_dir = 180 + 60;                      % deg clockwise from north
%Angle_wind_at6m = 180 + 90 - atan(1.728/2.932)*180/pi ;
Wind_dir = 30 + 180 ; 
Angle_wind_at6m = 180 + atan(1.728/2.932)*180/pi ;
V_wind_at6m = sqrt(2.932^2+1.728^2);      % m/s

%% Quadcopter physical parameters -- prsshin - hexacopter -- reduce the mass for a quad-compatibility

% dynamics related
m=1.035 * 4/6;                      % mass [kg]
l=0.225 ;                           % arm length [m]
Ix= 0.0469 ;
Iy= 0.0358 ;
Iz=  0.101 * 4/6;
I_B = diag([Ix,Iy,Iz]);
I_r = 3.357e-5;                     % rotor momemt of inertia (kg x m^2)

% aerodynamics related
b =  1.5652e-08 * (60/(2*pi))^2 ;   % Thrust coeffcient intially N/(rpm^2), now N/(rev/s^2)
k =  2.0862e-10 * (60/(2*pi))^2 ;   % Torque Coeffcient intially Nm/(rpm^2), now Nm/(rev/s^2)
R=3*0.0254;                         % propeller radius  [m]
nb=2;                               % number of blade
A=pi*R^2;                           % disk area
% sig=nb*mean(c)/(pi*R);              % solidity

% blade characteristics
rho = 1.15;
nr = 20;                             % number of radial points on a blade
npsi = 60;                            % number of azimuthal points for 2*Pi

%the0=repmat(5,[1,11]);
%cla1=repmat(1.8059*pi,[1,11]);
the0=repmat(4,[1,11]);
cla1=repmat(1.7059*pi,[1,11]);
th1=(the0+[24.9844849214694,24.4885730384207,23.6542985258914,22.4816610238794,20.9706612541160,19.1212982695717,16.9335723826041,14.4074838117246,11.5430326339323,8.34021849685304,4.79904143902087])*pi/180;
c1=[7.96284784614477,11.2448599794330,13.6346682267195,15.1322722373498,15.7376722802413,15.4508682842241,14.2718602574625,12.2006482106621,9.23723208187088,5.38161196932697,0.633787695366624]*0.001;

% interpolating data for different radius location (final size of the vectors 1*nr)
th=interp1(linspace(1/11,1,11),th1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
c=interp1(linspace(1/11,1,11),c1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');
cla=interp1(linspace(1/11,1,11),cla1,linspace(floor(0.15*nr)/nr,1,nr),'linear','extrap');

r=linspace(floor(nr*0.01)/nr,1,nr);    % normolized radial locations
psi=linspace(0,2*pi,npsi);             % azimuth angle

maxsize=max(nr,npsi);
numvar=11;
geometry2=zeros(numvar,maxsize);

list={R,nb,A,rho,nr,npsi,th,c,cla,r,psi};

for  i=1:numvar
    
geometry2(i,1:length(list{i}))=[list{i}];

end

geometry = Simulink.Signal;
geometry .DataType = 'double';
geometry .Dimensions = [length(list) npsi];
geometry .Complexity = 'real';
geometry .SamplingMode = 'Sample based';
geometry .InitialValue = 'geometry2';

%% Initial states

R_i = [0.0;0.0;0.0];
V_i = [0.0;0.0;-0.001];

% Initial orientation (Euler angles) w.r.t. the Earth inertial coordinate system
psi_i   = 0.0*dtr;
theta_i = 0.0*dtr;
phi_i   = 0.0*dtr;
InitialEulerAngles = [psi_i,theta_i,phi_i];

% initial body rates (rad/s)
Omega_i = [0;0;0];

%% Considered mission parameters -- ascend - cruise - descend
% phase 1
tf_phase1 = 10; %sec
% zf_phase1 = -60; % for circle
zf_phase1 = -40; % for circle

% phase2
length_phase2 = 1; %min
tf_phase2 = tf_phase1 + length_phase2*60;
xdot_cruise = 15; %m/s

%phase3
t_phase3 = 10; %sec
tsim = tf_phase2 + t_phase3;
deltaT = 15; % (sec) transition time from hover to cruise and cruise to hover

%% Control parameters
% Aosition control

Kp = diag([1,1,1]);
Kd = diag([2,2,2]);
Ki = diag([0.01,0.01,0.01]);
% Attitude control
Tbar = diag([15,15,15]);
Gama1 = diag([3,3,3]); 
Gama2 = diag([0.1,0.1,0.1]);
Lambda = diag([10,10,10]);
%    break

%% Run simulink file
sim('M_6DoF_BS_Full_backup_11_25_2018.slx')

%%
reset(gcf);reset(gca)
set(0,'defaultLineLineWidth',2)
set(0,'defaultAxesFontSize',12)

T_t = Pos.time;
Position = Pos.signals.values;
figure(1)
subplot(3,1,1)
plot(T_t,Position(:,1)/1000,'--b',T_t,Position(:,4)/1000,'r')
legend('Nominal','Actucal')
set(gca,'xticklabel',{[]})
ylabel('x (km)')
subplot(3,1,2)
plot(T_t,Position(:,2),'--b',T_t,Position(:,5),'r')
set(gca,'xticklabel',{[]})
ylabel('y (m)')
subplot(3,1,3)
plot(T_t,Position(:,3),'--b',T_t,Position(:,6),'r')
xlabel('Time (sec)')
ylabel('z (m)')

Velocities = Vel.signals.values;
figure(2)
subplot(3,1,1)
plot(T_t,Velocities(:,1),'--b',T_t,Velocities(:,4),'r')
legend('Nominal','Actucal')
set(gca,'xticklabel',{[]})
subplot(3,1,2)
plot(T_t,Velocities(:,2),'--b',T_t,Velocities(:,5),'r')
set(gca,'xticklabel',{[]})
subplot(3,1,3)
plot(T_t,Velocities(:,3),'--b',T_t,Velocities(:,6),'r')
ylabel('v_z (m/s)')
xlabel('Time (sec)')


figure(3)
subplot(3,1,1)
plot(Pos.time,Angles)
set(gca,'xticklabel',{[]})
ylabel('angles (deg)')
legend('\psi','\theta','\phi')
subplot(3,1,2)
plot(T_t,Omega)
set(gca,'xticklabel',{[]})
ylabel('\Omega (rad/s)')
legend('p','q','r')
subplot(3,1,3)
plot(T_t,Torques)
xlabel('Time (sec)')
ylabel('\tau (N m)')
legend('\tau_x','\tau_y','\tau_z')


figure(4)
plot(T_t,Rpms_fixed*60/(2*pi));hold on
plot(T_t,Rpms*60/(2*pi))
xlabel('Time (sec)')
ylabel('Propeller speed (rad/s)')
legend('\omega_1','\omega_2','\omega_3','\omega_4')

figure(5)
plot(T_t,Wind)
xlabel('Time (sec)')
ylabel('Wind speed (m/s)')
legend('w_x','w_y','w_z')

figure(6)
plot3(Position(:,1),Position(:,2),-Position(:,3),'--b')
hold on
plot3(Position(:,4),Position(:,5),-Position(:,6),'r')
xlabel('x (m)')
ylabel('y (m)')
zlabel('z (m)')
legend('Nominal trajectory','Actucal trajectory')

figure(7)
plot(T_t,Wind(:,3)./sqrt(sum(Wind(:,1:2).*Wind(:,1:2),2)))
xlabel('Time (sec)')
ylabel('$\frac{V_z}{\sqrt{v_x^2+v_y^2}}$','Interpreter', 'latex')
