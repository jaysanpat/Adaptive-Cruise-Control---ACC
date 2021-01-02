%% Controller Model Code

Wv = 3100;
Cd = 0.3;
Af = 18;
rho = 0.0024;
Ka = 10;
Kp = 10;
Ki = 50;
rw = 1;
g = 32.17;
theta = 0.05;
Vc = 88;
Kd = rho*Cd*Af*Vc;
gA = 4;
M = Wv/g;

%% Matlab Code 

% PI controller
num = [g 0];
den = [1 Kd/M+gA*Ka*Kp/(M*rw) Ka*Ki*gA/(M*rw)];
cruise_speed = tf(num,den);
T = 0:0.1:6;
a=zeros(1,20);
b=0.2*ones(1,41);
U=[a b];
% opt = stepDataOptions('InputOffset',60);
% step(-cruise_speed,opt,10)
sys=lsim(-cruise_speed, U, T)+60;
plot(sys)
ylabel('Velocity(mph)')
axis([0 60 59 61])


%% Using P Controller

% P controller only
ki=0;
den2 = [1 Kd/M+gA*Ka*Kp/(M*rw) Ka*ki*gA/(M*rw)];
cruise_speed2 = tf(num,den2);
sys2=lsim(-cruise_speed2, U, T)+60;
plot(sys2)
ylabel('Velocity(mph)')