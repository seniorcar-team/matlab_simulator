%経路図示ようプログラム
%ver1との変更点は時間ベースに変更したこと
%
%sは軌跡長，deltaは操舵角

% motion parameter
L = 0.9;  % wheel base
%ds = 0.1;  % course distanse

%n = round(s / ds);
time = [1.2,3.5];  % [s]　（スズキMTGのため）
dt = 0.1;
max_acceleration = 0.8;%[m/s^2]
n = round(time/dt);

n = round(time/dt);

state = zeros(1,5);
K = 0:deg2rad(10):deg2rad(40);
ref_speed = [1.0/3.6,6.0/3.6];
for k = 1:length(K)
    for j = 2:2
        x = zeros(n(j)+1,1);
        y = zeros(n(j)+1,1);
        v = zeros(n(j)+1,1);
        yaw = zeros(n(j)+1,1);
        kp = K(k)*ones(n(j)+1,1);
        state=[0,0,0,0,kp(1)];
        t = 0:0.1:time(j);
        for i = 2:n(j)+1
            v(i) = max([0,min([ref_speed(j),0+max_acceleration*(2*i-1)*dt/2])]);
            state = update(state, v(i), kp(i), dt, L);
            x(i+1) = state(1);
            y(i+1) = state(2);
            yaw(i+1) = state(3);
        end
        plot(x,y,'r');hold on;
        if k == 1
        x(n(j)+1)
        end
    end
end
length(v)
%{
v_1 = [0,0,0.05,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]/3.6;
for k = 1:length(K)
    state=[0,0,0,0,kp(1)];
    
    kp = K(k)*ones(n(1)+1,1);
    x = zeros(n(1)+1,1);
    y = zeros(n(1)+1,1);
    v = zeros(n(1)+1,1);
    yaw = zeros(n(1)+1,1);
    t = 0:0.1:time(1);
    for i = 1:n(1)+1
        state=update(state,v_1(i),kp(i),dt,L);
        x(i+1) = state(1);
        y(i+1) = state(2);
        yaw(i+1) = state(3);
    end
    if k == 1
        x(n(1)+1)
    end
    plot(x,y,'ok');hold on;
end
%}


v_6=[0,0,0.12,0.4,0.5,1.0,1.25,1.5,1.8,2.25,2.5,2.75,3.0,3.4,3.6,4.0,4.2,4.3,4.5,4.6,4.7,4.75,4.8,4.9,5.0,5.1,5.2,5.25,5.3,5.4,5.5,5.5,5.6,5.75,5.8,6]/3.6;
for k = 1:length(K)
    state=[0,0,0,0,kp(1)];
    kp = K(k)*ones(n(2)+1,1);
    x = zeros(n(2)+1,1);
    y = zeros(n(2)+1,1);
    v = zeros(n(2)+1,1);
    yaw = zeros(n(2)+1,1);
    t = 0:0.1:time(2);
    for i = 1:n(2)+1
        state=update(state,v_6(i),kp(i),dt,L);
        x(i+1) = state(1);
        y(i+1) = state(2);
        yaw(i+1) = state(3);
    end
    if k == 1
        x(n(2)+1)
    end
    plot(x,y,'ok');hold on;
end

%{
hold on;

acceleration(j)_array = tan(kp).*v.^2/0.9;
plot(acceleration(j)_array);
%}
%{
x = zeros(n+1,1);
y = zeros(n+1,1);
yaw = zeros(n+1,1);
x(1) = state(1);
y(1) = state(2);
yaw(1) = state(3);

for i=1:n
    state = update(state, v(i), kp(i), dt, L);
    x(i+1) = state(1);
    y(i+1) = state(2);
    yaw(i+1) = state(3);
end
path_array = [ x, y, yaw,v,kp];
%plot(x,y);hold on;
end
%}
function state_array = update(state, v, kp, dt, L)
state_array = zeros(1,4);
state_array(4) = v;
state_array(5) = kp;

if kp~= 0
    turning_radius = L/tan(kp);
else
    turning_radius = 100000;
end
r_x = turning_radius*sin(v*dt/turning_radius);
r_y = turning_radius*(1-cos(v*dt/turning_radius));
transition = rot([r_x,r_y],state(3));
state_array(1) = state(1) + transition(1);
state_array(2) = state(2) + transition(2);
state_array(3) = state(3) + v/turning_radius * dt;
%state_array(3) = pi_2_pi(state_array(3));

%{
state_array(1) = state(1) + v * cos(state(3)) * dt;
state_array(2) = state(2) + v * sin(state(3)) * dt;
state_array(3) = state(3) + v / L * tan(kp) * dt;
%}
end

function angle = pi_2_pi(rad)
    angle = rad;
    if rad > 2*pi
        a = floor(rad/(2*pi));
        angle = angle - 2.0 * pi *a;
    end
    if rad < -2*pi
        a = ceil(abs(rad)/(2*pi));
        angle = angle + 2.0 * pi * a;
    end
    
    if angle > pi
        angle = angle - 2*pi;
    end
    
    if angle < -pi
        angle = angle + 2*pi;
    end
end

