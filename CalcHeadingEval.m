%% heading�̕]���֐����v�Z����֐�
function heading=CalcHeadingEval(x,goal)

theta=Pi2Pi(x(3));%���{�b�g�̕���
goalTheta=atan2(goal(2)-x(2),goal(1)-x(1));%�S�[���̕���

targetTheta=abs(Pi2Pi(goalTheta-theta));%�S�[���܂ł̕��ʍ���[deg]

heading=pi-targetTheta;