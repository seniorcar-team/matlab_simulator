function vel_sp = calc_vel_sp(x,Pedestrian,xt)
%DWA－CSCに関する評価関数，速度に関する罰則を与える．
%現在の衝突可能性のある対象が速度を保ったまま進むとき，ちょうど衝突するときの速度を最大として，それに近いほど罰則が大きい
%この関数の返り値にκマックスをかける
vel_sp = 0;
for i = 1:length(Pedestrian)
    Pedestrian_position = rot([Pedestrian(i).tmp_position(1)-x(1),Pedestrian(i).tmp_position(2)-x(2)],-x(3));%車両座標系(現在の車両の位置及び姿勢に平行移動と回転をさせたもの)
    xt_position = rot([xt(1)-x(1),xt(2)-x(2)],-x(3));
    Pedestrian_direction = atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1))-x(3);
    Pedestrian_velocity = rot([Pedestrian(i).velocity(1),Pedestrian(i).velocity(2)],-x(3));%車両座標系(現在の車両の位置及び姿勢に平行移動と回転をさせたもの)
    CSC = 0;
    %衝突可能性判定
    if Pedestrian_position(1) < 0
        continue;
    end
    if Pedestrian_position(2) > 0
        if atan2(-Pedestrian_position(2),-Pedestrian_position(1))...
                <= Pedestrian_direction && ...
                Pedestrian_direction < 0
            CSC =1;
        else
            continue;
        end
    else
        if 0 <= Pedestrian_direction && ...
            atan2(-Pedestrian_position(2),-Pedestrian_position(1)) > Pedestrian_direction
        CSC =1;
        else
            continue;
        end
    end
    if CSC ==1
        gradient = tan(atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1))-x(3));
        D = abs(gradient*(xt_position(1)-Pedestrian_position(1))-1*(xt_position(2)-Pedestrian_position(2)))...
            /sqrt(gradient^2+1^2);%歩行者の進行方向に基づく直線からの距離
        vel_max = norm([Pedestrian(i).velocity(1),Pedestrian(i).velocity(2)])*...
            (-1*Pedestrian_position(2)/(tan(Pedestrian_direction))+Pedestrian_position(1))...
            /(-1*Pedestrian_position(2)/(tan(Pedestrian_direction))*sqrt(1+tan(Pedestrian_direction)^2));%衝突点に歩行者と同じタイミングで入る速度
        vel_sp = vel_sp+D*abs(Pedestrian_velocity(2))/vel_max;%元論文ではこうなってたけど割っているのがロボットの最高許容速度で割られるのが歩行者の速度の動作延長上への垂直方向成分なのが謎…
    else 
        continue;
    end
end

