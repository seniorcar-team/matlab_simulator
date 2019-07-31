%% 距離項の算出（歩行者は現在位置を利用）
function [dist,ind]=CalcDistEval(x,Pedestrian,dist_max)
%歩行者との距離評価値を計算する関数
for io=1:length(Pedestrian)
    disttmp(io)=norm(Pedestrian(io).tmp_position-x(1:2)');%パスの位置と障害物とのノルム誤差を計算
end
[dist_ob,ind] = min(disttmp);
dist = min([dist_max, dist_ob]);