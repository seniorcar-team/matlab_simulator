function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
%�O�Ճf�[�^���쐬����֐�
global dt;
time=0;
u=[vt;ot];%���͒l
% traj=x;%�O�Ճf�[�^
traj = zeros(5,round(evaldt/dt)+1);
traj(:,1) = x;
i = 1;
while time<=evaldt
    time=time+dt;%�v�Z���Ԃ̍X�V
    i = i + 1;
    x=f_sim(x,u);%�^�����f���ɂ�鐄��
%     traj=[traj x];
    traj(:,i) = x;
end