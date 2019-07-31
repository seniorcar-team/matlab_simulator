function path_array = motion_model_base_trajectory_genarator_complete_ver(target_speed,delta_middle,delta_finish,delta_start,x)
%車両のモーター特性を考慮した形に変更，加速度の制約はlook up table 作成時に反映 2019_3_11

%sは軌跡長，deltaは操舵角

% motion parameter
L = 0.9;  % wheel base
%ds = 0.1;  % course distanse
MAX_MOTOR_ACCELERATION = 3.0/3.6;%[m/s]
MIDDLE_LEVEL_MOTOR_ACCELERATION = 1.0/3.6;%[m/s]
MIDDLE_LEVEL_MOTOR_DECELERATION = -5.0/3.6;%[m/s]
MAX_MOTOR_DECELERATION = -8.0/3.6;%[m/s]

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
reference_speed = zeros(n+1,1);
state = [x(1),x(2),x(3),x(4),x(5)];
delta_gradient_1 = (delta_middle-delta_start)/(time/2);
delta_gradient_2 = (delta_finish-delta_middle)/(time/2);
intercept = delta_finish - delta_gradient_2*time;
current_speed = state(4);
v(1) = x(4);
kp(1) = x(5);
abs_max_jerk = 0.7;
%acceleration = tan(delta)*v^2/0.9;
%atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*0.1)/(v(i)^2/0.9))>= kp(i)=atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*0.1)/(v(i)^2/0.9));
%if current_speed is zero,then some delay is occured 0.15s
%Only using reference speed,position error is approximately 20cm during acceleration from 0km/h to 6km/h
for i = 2:n+1
    reference_error = target_speed - reference_speed(i-1);
    if reference_error  >= 2/3.6
        manipulative_variable = MAX_MOTOR_ACCELERATION  / Frequency;%combine acceleration and time by multiplication.
    else if reference_error > 0 && reference_error < 2/3.6
        manipulative_variable = MIDDLE_LEVEL_MOTOR_ACCELERATION  / Frequency;
    else if reference_error == 0
        manipulative_variable = 0;
    else if reference_error > -2/3.6 && reference_error < 0
        manipulative_variable = MIDDLE_LEVEL_MOTOR_DECELERATION  / Frequency;
    else
        manipulative_variable = MAX_MOTOR_DECELERATION / Frequency;
    end
    reference_speed(i) = reference_speed(i-1) + manipulative_variable;
    %車速0.5㎞/hまでは平均的にMIDDLE_LEVEL_MOTOR_ACCELERATIONの速度で追従，Referenceとの差分が0.2km/h以下でMIDDLE_LEVEL_MOTOR_ACCELERATION，それ以上でMAX_MOTOR_ACCELERATION，target_speedに達した場合はその速度とする.
    %減速域のデータが無いので減速域は例外的にレファレンスに0.2秒遅れているとして扱う
    speed_error = current_reference - current_speed;
    if speed_error > 0 && current_speed < 0.5/3.6
        response = MIDDLE_LEVEL_MOTOR_ACCELERATION  / Frequency;
    else if speed_error == 0
        response = 0;
    else if speed_error > 0 && speed_error <= 0.2/3.6
        response = MIDDLE_LEVEL_MOTOR_ACCELERATION  / Frequency;
    else if speed_error > 0.2/3.6
        response = MAX_MOTOR_ACCELERATION  / Frequency;
    end
    if speed_error >= 0
        v(i) = v(i-1) + response;
    else if speed_error < 0 && i<=2
        v(i) = 0;
    else
        v(i) = reference_speed(i-2);
    end
    % kp は　走行中の応答状態を把握してから設計，加速度の制約は無しで，あとで呼び出すプログラムにおいて指標の確認
    if i <= 1 + n/2  
        kp(i) = pi_2_pi(max([-abs_max_steer_angle,min([delta_gradient_1*(i-1)/10+delta_start,abs_max_steer_angle])]));%ステップごとの操舵角
        kp(i) = min([atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*dt)/(v(i)^2/0.9)),...
           max([kp(i),atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*dt)/(v(i)^2/0.9))])]);
    else
        kp(i) = pi_2_pi(max([-abs_max_steer_angle,min([delta_gradient_2*(i-1)/10+intercept,abs_max_steer_angle])]));
        kp(i) = min([atan((tan(kp(i-1))*v(i-1)^2/0.9+abs_max_jerk*dt)/(v(i)^2/0.9)),...
           max([kp(i),atan((tan(kp(i-1))*v(i-1)^2/0.9-abs_max_jerk*dt)/(v(i)^2/0.9))])]);
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

