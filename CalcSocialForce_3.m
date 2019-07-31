%% ���s�҂炵���̎Z�o�iSocialForce�ɂ��v�Z�j
function Sfish=CalcSocialForce_3(Sf,x,u_target)
global maximum_deceleration;  %�ō������x[m/s2]
global maximum_steering_velocity; %�ō����Ǌp���x[rad/s]
%Social Force�Ƃ̃}�b�`�x���v�Z����֐�
v_des = norm(Sf);
delta_des = Pi2Pi(atan2(Sf(2),Sf(1))-x(3));
manhattan = abs(v_des - u_target(1))/maximum_deceleration...
    + abs(Pi2Pi(delta_des - u_target(2)))/maximum_steering_velocity;

Sfish = 1/(1+manhattan);
