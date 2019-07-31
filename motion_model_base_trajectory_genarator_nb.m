function path_array = motion_model_base_trajectory_genarator_nb(acceleration,delta_middle,delta_finish,delta_start,x)
%ver1との変更点は時間ベースに変更したこと
%
%sは軌跡長，deltaは操舵角

% motion parameter
L = 0.9;  % wheel base
%ds = 0.1;  % course distanse

%n = round(s / ds);
time = 1.4;  % [s]
dt = 0.1;
n = round(time/dt);
%s = v*time;
%tk = [0.0, time / 2.0, time];
%kk = [delta_start, delta_middle, delta_finish];
%t = linspace(0.0, time, n+1);
kp = zeros(n+1,1);
v = zeros(n+1,1);
state = [x(1),x(2),x(3),x(4),x(5)];
delta_gradient_1 = (delta_middle-delta_start)/(time/2);
delta_gradient_2 = (delta_finish-delta_middle)/(time/2);
intercept = delta_finish - delta_gradient_2*time;
v(1) = x(4);
kp(1) = x(5);
abs_max_steer_angle = deg2rad(35);
%acceleration = tan(delta)*v^2/0.9;
%atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*0.1)/(v(i)^2/0.9))>= kp(i)=atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*0.1)/(v(i)^2/0.9));
for i = 2:n+1
    v(i) = max([0,min([1.6,x(4)+acceleration*(2*i-1)*dt/2])]);
    %abs_max_steer_angle = min([deg2rad(35),atan(0.45/(v(i)^2))]);%加速度値が0.5[m/s^2]を超えないような制限
    if i <= 1 + n/2  
        kp(i) = pi_2_pi(max([-abs_max_steer_angle,min([delta_gradient_1*(i-1)/10+delta_start,abs_max_steer_angle])]));%ステップごとの操舵角
        %kp(i) = min([atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*dt)/(v(i)^2/0.9)),...
        %   max([kp(i),atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*dt)/(v(i)^2/0.9))])]);
    else
        kp(i) = pi_2_pi(max([-abs_max_steer_angle,min([delta_gradient_2*(i-1)/10+intercept,abs_max_steer_angle])]));
        %kp(i) = min([atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*dt)/(v(i)^2/0.9)),...
        %   max([kp(i),atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*dt)/(v(i)^2/0.9))])]);
    end
    
end
%{
hold on;

acceleration_array = tan(kp).*v.^2/0.9;
plot(acceleration_array);
%}
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

