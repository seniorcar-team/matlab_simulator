%% 壁との距離項の計算
function dist_ob=CalcDistObstacleEval(x,ob,dist_max)
%壁との距離評価値を計算する関数
for io=1:length(ob)
    di_ob(1)=ob(io,1)-x(1);
    di_ob(2)=ob(io,2)-x(2);
    dist_ob_set(io)=norm(di_ob);%パスの位置と障害物とのノルム誤差を計算
end
dist_ob = min(dist_ob_set);
dist_ob = min([dist_max, dist_ob]);