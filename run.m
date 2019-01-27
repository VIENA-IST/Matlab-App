clc
clear all
close all

g=9.81;

% induction motor
pp=2;
V_F=1; % V/f ratio

% Modelo FIAT Cinquencento
Cd=0.33; % FIAT Cinquecento wikipedia https://en.wikipedia.org/wiki/Fiat_Cinquecento
A= 1.508*1.420; % area from wikipedia
m=700+2*70; %peso do carro + 2 pessoas
rw=0.28;
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

alfa_v=1*[0 0 3 3 0 0 3 3]*pi/180;
alfa_t=[0 0.1 0.101 0.5 0.501 0.8 0.801 1]*t_max;

theta_v=[0 0]*pi/180;
theta_t=[0 1]*t_max;

v_v=1*[0 500 500 1000 -1000 -500 0];
v_t=[0 0.1 0.3 0.4 0.6 0.7 1]*t_max;

% sim('model_1.slx')

motor = VIENAGUI.InductionMachine;

EQSolutions = @(v,f,r) motor.EQSolutions(v,f,r);

%%
sim('full_model_car.slx')


%%
figure
plot(x_v0.Data,y_v0.Data)
axis equal
hold on
h3=plot(x_v0.Data(2,1),y_v0.Data(2,1),'ro');

for i=1:length(x_v0.time)
    h3.XData=x_v0.Data(i);
    h3.YData=y_v0.Data(i);
    drawnow
    pause(0.05)
end
