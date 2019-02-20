clc
clear all
close all

g=9.81;

% induction motor
pp=2;
V_F=1; % V/f ratio

% Modelo FIAT Cinquecento
Cd=0.33; % FIAT Cinquecento wikipedia https://en.wikipedia.org/wiki/Fiat_Cinquecento
A= 1.508*1.420; % area from wikipedia
m=700+2*70; %peso do carro + 2 pessoas
rw=0.1651;
Izz=1/2*m*(3.23/2)^2;
lf=3.23/2;
lr=3.23/2;

% wheel
fr=0.01; % rolling resistance coeff.
m_wheel=3;
Iw=1/2*m_wheel*rw^2; % or m?
cp=60; %cornering stifness of tire

% parametros
rho_air=[1.2922 1.2466 1.2041 1.1644; 0 10 20 30]; % rho(temp_air)

Tar=273;
Press=101325;
R=287.058;
rho=Press/(R*Tar);
nhu=0.9;
Fz=m*g;

%% pista + torque

t_max=300;
t_sample=1;

theta_v=[5 5]*pi/180;
theta_t=[0 1]*t_max;

v_v=5000*[0 0.5 1]; % rpm
v_t=[0 0.1 1]*t_max;

motor = VIENAGUI.InductionMachine;

EQSolutions = @(v,f,r) motor.EQSolutions(v,f,r);

%% follow path
% global steering;
steering = SteeringController();
steering.create_map();

% starting point of car
car = steering.map_points(1,:) + 2*rand(1,2);

figure(1)
[x0, y0]=ginput(1);
plot(x0,y0,'r+')
[x2, y2]=ginput(1);
psi0 = atan2(-(x2-x0),(y2-y0));

endpoint = steering.map_points(end,:);
dt=0.1;
steering.sampling_time = dt;
wheel_base = 2.2;
wheel_angle = 0;

%%
sim('viena_model_R2017b_v2.slx')

hold on
plot(x_v0.Data,y_v0.Data)

