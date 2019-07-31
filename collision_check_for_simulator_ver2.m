function is_collision=collision_check_for_simulator_ver2(New_node,pedestrian_position_matrix)
%将来的にぶつかるしかないケースも衝突と判定する
%pedestrian_position_matrixは(201,41,41)三次元空間上に存在確率が格納されている
x_counter = size(pedestrian_position_matrix,1);
y_counter = size(pedestrian_position_matrix,2);
t_counter = size(pedestrian_position_matrix,3);
is_collision = 0;
global road_width;
%threshold = 0.005;
%シミュレーション結果によって変更,時間変化させる必要がある
%衝突領域を車両中心に対して設定する，衝突領域は長方形をさらに歩行者半径分拡大したもの
pedestrian_radius = 0.2+0.1;%観測誤差分のマージンを加えた
vehicle_width = 650*10^-3;
vehicle_length = 1100*10^-3;
vehicle_front_length = 900*10^-3;
vehicle_rear_length = vehicle_length - vehicle_front_length;
%車両座標系での衝突領域の頂点
x_car(1) = vehicle_front_length+0.3+pedestrian_radius;
y_car(1) = vehicle_width/2+pedestrian_radius;
x_car(2) = -(vehicle_rear_length+pedestrian_radius);
y_car(2) = vehicle_width/2+pedestrian_radius;
x_car(3) = -(vehicle_rear_length+pedestrian_radius);
y_car(3) = -(vehicle_width/2+pedestrian_radius);
x_car(4) = vehicle_front_length+0.3+pedestrian_radius;
y_car(4) = -(vehicle_width/2+pedestrian_radius);
%グローバル座標での車両の4点
car_point = zeros(4,2);
car_point(1,1) = New_node(1)+(vehicle_front_length)*cos(New_node(3))+vehicle_width/2*cos(New_node(3)+pi/2);
car_point(1,2) = New_node(2)+(vehicle_front_length)*sin(New_node(3))+vehicle_width/2*sin(New_node(3)+pi/2);
car_point(2,1) = New_node(1)+(vehicle_front_length)*cos(New_node(3))+vehicle_width/2*cos(New_node(3)-pi/2);
car_point(2,2) = New_node(2)+(vehicle_front_length)*sin(New_node(3))+vehicle_width/2*sin(New_node(3)-pi/2);
car_point(3,1) = New_node(1)-vehicle_rear_length*cos(New_node(3))+vehicle_width/2*cos(New_node(3)+pi/2);
car_point(3,2) = New_node(2)-vehicle_rear_length*sin(New_node(3))+vehicle_width/2*sin(New_node(3)+pi/2);
car_point(4,1) = New_node(1)-vehicle_rear_length*cos(New_node(3))+vehicle_width/2*cos(New_node(3)-pi/2);
car_point(4,2) = New_node(2)-vehicle_rear_length*sin(New_node(3))+vehicle_width/2*sin(New_node(3)-pi/2);
turning_radius = 0.9/tan(deg2rad(35));
turning_center_counter_clockwise_y_position = New_node(2) + turning_radius*sin(New_node(3)+pi/2);
turning_center_clockwise_y_position = New_node(2) + turning_radius*sin(New_node(3)-pi/2);
turning_radius_2 = 1.9;%最外側が描く円の半径
collision_check_point = zeros(2,1);
collision_check_point(1) = turning_center_counter_clockwise_y_position - turning_radius_2;
collision_check_point(2) = turning_center_clockwise_y_position + turning_radius_2;
if New_node(2)<0 && collision_check_point(1)< -road_width/2
    is_collision = 1;
    %disp('circle');
    return;
elseif New_node(2)>=0 && collision_check_point(2)> road_width/2
    is_collision = 1;
    %disp('circle');
    return;
end
if max(abs(car_point(:,2)))>road_width/2
    is_collision = 1;
    %disp('4point');
    %disp('!');
    return;
end

x_range = x_car(1)-x_car(3);
y_range = y_car(1)-y_car(3);
x_step_num = ceil(x_range*10)+1;
y_step_num = ceil(y_range*10)+1;
time_ind=max([1,min([t_counter,ceil(New_node(4)*10+1)])]);
%{
max_p =max(reshape(pedestrian_position_matrix(:,:,round(time_ind)),[],1));
min_p =min(reshape(pedestrian_position_matrix(:,:,round(time_ind)),[],1));
%}
threshold = 0.6*max(reshape(pedestrian_position_matrix(:,:,round(time_ind)),[],1));

for i = 1:x_step_num
    for j=1:y_step_num

        x=x_car(3)+0.1*(i-1);
        if i ==x_step_num
            x=x(1);
        end
        y=y_car(3)+0.1*(j-1);
        if j==y_step_num
            y=y(1);
        end
        
        x_1 = rot([x,y],New_node(3));
        x_global = New_node(1)+x_1(1);
        y_global = New_node(2)+x_1(2);
        x_ind = max([1,min([x_counter,round(x_global,1)*10 + 1])]);
        y_ind = max([1,min([y_counter,(round(y_global,1)+2)*10 + 1])]);
        

        if pedestrian_position_matrix(round(x_ind),round(y_ind),round(time_ind))>threshold%0.1117*exp(-0.845*(time_ind-21)/10)/2
            is_collision = 1;
            %disp('person')
            return
        end
        
    end
end
        
        
        
        