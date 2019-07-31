function dir_sp = calc_dir_sp(x,Pedestrian,xt)
%DWA-CSC用の評価関数，角度に関する罰則を与える
%この関数の返り値にθマックスをかける，θマックスは負の値？
dir_sp =0;
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
        theta = (pi-abs(Pi2Pi(x(3)-atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1)))))/pi;%現状対立方向だと低い値，論文中ではDWAは同じ方向の時に引っかかりやすいとの話があったので，抑制項をどれだけ強く働かせるかの係数？
    else
        continue;
    end    
    dir_sp = dir_sp+theta*Pi2Pi(xt(3)-atan2(Pedestrian(i).velocity(2),Pedestrian(i).velocity(1)));%元論文ではこうなってたけど片方の角度にしか正規化してないのと，なす角がπの時評価が最低になるで統一していないのが謎…
end

