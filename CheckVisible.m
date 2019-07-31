%% 歩行者がセンシング範囲かを判定する
function Pedestrian_new = CheckVisible(x, Pedestrian_new)

global pedestrian_width;
global vision_distance_senior;

global vehicle_length;
global vehicle_rear_length;

vehicle_to_center = vehicle_length/2 - vehicle_rear_length;

Pedestrian_new = Pedestrian_new;

position_data = zeros(length(Pedestrian_new),1);
angle_data = zeros(length(Pedestrian_new),1);
angle_range_data = zeros(length(Pedestrian_new),1);

x_seniorcar = [x(1)+ vehicle_to_center*cos(x(3)) x(2)+ vehicle_to_center*sin(x(3))];

for num = 1:length(Pedestrian_new)
    relative_position = Pedestrian_new(num).tmp_position - x_seniorcar;
    position_data(num) = norm(relative_position);
    angle_data(num) = atan2(relative_position(2),relative_position(1))-x(3);%修正20181014
    Pedestrian_new(num).visible = 0;
end

[B,I] = sort(position_data);

for num = 1:length(Pedestrian_new)
%     if num~=1
%         Pedestrian_new(I(num-1)).visible
%     end
    if norm(position_data(I(num))) <= vision_distance_senior
        if num~=1
            for j = 1:num-1
                if angle_data(I(j)) - asin(pedestrian_width/position_data(I(j))) <= angle_data(I(num)) && ...
                    angle_data(I(num)) <= angle_data(I(j)) + asin(pedestrian_width/position_data(I(j)))
                    Pedestrian_new(I(num)).visible = 0;
%                     disp(Pedestrian_new(I(num)).visible);
                    break;
                else
                    
                    Pedestrian_new(I(num)).visible = 1;
%                     disp('hello');
                    if abs(Pi2Pi(angle_data(I(num))))>pi/2
                        Pedestrian_new(I(num)).visible = 0;
                    end
                end
            end
        else
            Pedestrian_new(I(num)).visible = 1;
            if abs(Pi2Pi(angle_data(I(num))))>pi/2
                Pedestrian_new(I(num)).visible = 0;
            end
        end
    else
        break;
    end
end
% pause;