function [u,trajDB,evalDB,ind] = ...
        DynamicWindowApproach_CSC(x,Kinematic,goal,Pedestrian,PARAM)
ind = 0;
%Dynamic Window[vmin,vmax,ωmin,ωmax]の作成
Vr=CalcDynamicWindow(x,Kinematic);

[evalDB,trajDB]=Evaluation_DWA_CSC(x,Vr,goal,Kinematic,PARAM,Pedestrian);%この関数は後で作る2018/10/12.16:23

if isempty(evalDB)
    disp('no path to goal!!');
    u=[0;0];return;
end

if ~any(evalDB(:,3))
   if(x(4)~=0)
%        disp('no control value4');
       u=[Vr(1);x(5)];return;
%        pause
   end
%    pause
end
%各評価関数の正規化
evalDB=NormalizeEval_DWA_CSC(evalDB);%この関数は後で作る2018/10/12.16:23

%最終評価値の計算
feval=[];
for id=1:length(evalDB(:,1))
    feval=[feval;PARAM(1:5)*evalDB(id,3:7)'];
end
evalDB=[evalDB feval];
% disp(evalDB);
[~,ind]=max(feval);%最も評価値が大きい入力値のインデックスを計算
uu=evalDB(ind,1:2)';%評価値が高い入力値(速度，操舵角)を返す
if uu(1) <= 0.1
    uu(1)=uu(1);
    uu(2)= x(5);
    u = uu;
%     disp('hello');
else
%     disp(evalDB(ind,:));
    u=uu;
end