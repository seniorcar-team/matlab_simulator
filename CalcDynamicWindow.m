%% 車両モデルの制約の算出
function Vr=CalcDynamicWindow(x,model)
%モデルと現在の状態からDyamicWindowを計算
% global dt;
target_dt = 0.1;
%車両モデルによるWindow
Vs=[0 model(1) -model(2) model(2)];

%運動モデルによるWindow
Vd=[x(4)-model(3)*target_dt x(4)+model(3)*target_dt x(5)-model(4)*target_dt x(5)+model(4)*target_dt];

%最終的なDynamic Windowの計算
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]
