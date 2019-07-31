function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
%軌跡データを作成する関数
global dt;
time=0;
u=[vt;ot];%入力値
% traj=x;%軌跡データ
traj = zeros(5,round(evaldt/dt)+1);
traj(:,1) = x;
i = 1;
while time<=evaldt
    time=time+dt;%計算時間の更新
    i = i + 1;
    x=f_sim(x,u);%運動モデルによる推移
%     traj=[traj x];
    traj(:,i) = x;
end