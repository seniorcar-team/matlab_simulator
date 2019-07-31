%% 歩行者らしさの算出（SocialForceによる計算）
function Sfish=CalcSocialForce_3(Sf,x,u_target)
global maximum_deceleration;  %最高減速度[m/s2]
global maximum_steering_velocity; %最高操舵角速度[rad/s]
%Social Forceとのマッチ度を計算する関数
v_des = norm(Sf);
delta_des = Pi2Pi(atan2(Sf(2),Sf(1))-x(3));
manhattan = abs(v_des - u_target(1))/maximum_deceleration...
    + abs(Pi2Pi(delta_des - u_target(2)))/maximum_steering_velocity;

Sfish = 1/(1+manhattan);
