%% 距離項の算出（歩行者は現在位置を利用）
function [dist_ped ind]=CalcDistEvalAll(traj,Pedestrian,dist_max)
%歩行者との距離評価値を計算する関数

dist_ped = 10;
ind = 1;
for i = 1:length(traj(1,:))
    for io=1:length(Pedestrian)
        disttmp(1) = Pedestrian(io).tmp_position(1)-traj(1,i);
        disttmp(2) = Pedestrian(io).tmp_position(2)-traj(2,i);
        if norm(disttmp) < dist_ped
            dist_ped = norm(disttmp);
            ind = io;
        end
    end
end
