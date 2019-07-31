function predict_position_matrix_abs_cart = predict_position_for_simulator_ver3(occupancy_matrix,predict_position_matrix_abs_cart_pre,x_seniorcar,Pedestrian)
%代表位置のカーテシアンの絶対座標と観測された時刻が記録されている
%歩行者番号，方向，現在のθ，ｒ，現在のx，y，0.1秒前のx，y．．．，2秒前のx，y
ped_num = size(predict_position_matrix_abs_cart_pre,1);
predict_position_matrix_abs_cart = zeros(ped_num,46);
predict_position_matrix_abs_cart(:,7:46) = predict_position_matrix_abs_cart_pre(:,5:44);%ひとつ前のものは今からみたら0.1秒前の位置になるから2個ずらして移す
angle_increment = 0.25/180*pi;
for i = 1:ped_num
    A = occupancy_matrix(1,:);%角度ごとの観測された歩行者が格納されている
    observed_ind_array = find(A == i);
    if abs(deg2rad(Pedestrian(i).direction))<pi/2
        direction = 1;%セニアカーと同じ方向にゴールを持っていると考える，画像ベースで進行方向がどの向きかを二元的に考えることは可能
    else
        direction = 0;
    end
    
    if isempty(observed_ind_array)
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    
    elseif occupancy_matrix(2,observed_ind_array(1))*angle_increment*(length(observed_ind_array)-1)< 0.1
        %距離×観測できた角度の範囲の大きさ≒観測できた部分の長さ
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    observable_point_num = length(observed_ind_array);
    x = zeros(1,observable_point_num);
    y = zeros(1,observable_point_num);
    for j = 1:observable_point_num        
        [x(j),y(j)] = pol2cart(-pi/2+angle_increment*(min(observed_ind_array)+(j-1)-1),occupancy_matrix(2,min(observed_ind_array)+j-1));
    end
    
    C0 = [mean(x),mean(y)]; 
    f = @(C) norm((x-C(1)).^2 + (y-C(2)).^2-0.2^2);
    Cf1 = fminsearch(f,C0);%代表点
    if isreal(Cf1(1))~= 1
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    if isreal(Cf1(2))~= 1
        predict_position_matrix_abs_cart(i,1:6)  = [i,direction,NaN,NaN,NaN,NaN];
        continue
    end
    [representative_theta,representative_range]=cart2pol(Cf1(1),Cf1(2));
    [represented_x,represented_y]=...
            pol2cart(representative_theta+x_seniorcar(3),representative_range);

        
    predict_position_matrix_abs_cart(i,1:6) = ...
    [i,direction,representative_theta,representative_range,x_seniorcar(1)+0.9*cos(x_seniorcar(3))+represented_x...
    ,x_seniorcar(2)+0.9*sin(x_seniorcar(3))+represented_y];
end
