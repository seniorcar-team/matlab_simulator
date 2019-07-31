%% 歩行者との衝突予測(予測時間までの車両軌道が現在の歩行者の現在位置と衝突しないかの判定を行う)
function safe_ped = Judgement_margin_2_time(traj, Pedestrian)

global vehicle_length;
global vehicle_width;
global vehicle_front_length;
global vehicle_rear_length;
global pedestrian_width;
global safety_margin_front;
global safety_margin_side;
global safety_margin_rear;
global maximum_deceleration; 
global dt;

carbody_edge = [vehicle_front_length+safety_margin_front -vehicle_width/2-safety_margin_side;
                vehicle_front_length+safety_margin_front vehicle_width/2+safety_margin_side;
               -vehicle_rear_length-safety_margin_rear -vehicle_width/2-safety_margin_side;
               -vehicle_rear_length-safety_margin_rear vehicle_width/2+safety_margin_side];

safe_ped =1;
% for i = 1:length(traj(1,:))
for i = 1:ceil(traj(4,2)/maximum_deceleration/dt)+3
    if safe_ped == 1;
        for j = 1:length(Pedestrian)
            if Pedestrian(j).visible
                
            % relative_position
            relative_position(1) = Pedestrian(j).tmp_position(1) - traj(1,i);
            relative_position(2) = Pedestrian(j).tmp_position(2) - traj(2,i);
            if norm(relative_position) > pedestrian_width ...
                   + ((vehicle_front_length+safety_margin_front)^2 + (vehicle_width/2+safety_margin_side)^2)^0.5 
                continue
            end
            % rotation
            relative_position_rota(1) =  relative_position(1)*cos(traj(3,i)) + relative_position(2)*sin(traj(3,i));
            relative_position_rota(2) =  -relative_position(1)*sin(traj(3,i)) + relative_position(2)*cos(traj(3,i));
            % judgement
            
            % 歩行者が車両のボデーの中にいれば衝突と判定
            if relative_position_rota(1) < vehicle_front_length + safety_margin_front + pedestrian_width ...
                    && relative_position_rota(1) > - vehicle_rear_length  - safety_margin_rear - pedestrian_width
                if relative_position_rota(2) < vehicle_width/2 + safety_margin_side ...
                        && relative_position_rota(2) > - vehicle_width/2 - safety_margin_side 
                    safe_ped = 0;
%                     disp('collide front')               
                    break;
                end
            end
            
            if relative_position_rota(1) < vehicle_front_length + safety_margin_front ...
                    && relative_position_rota(1) > - vehicle_rear_length  - safety_margin_rear
                if relative_position_rota(2) < vehicle_width/2 + safety_margin_side + pedestrian_width ...
                        && relative_position_rota(2) > - vehicle_width/2 - safety_margin_side - pedestrian_width
                    safe_ped = 0;
%                     disp('collide front')               
                    break;
                end
            end
            
            for i=1:length(carbody_edge(1,:))
               if norm(relative_position_rota-carbody_edge(i,:)) < pedestrian_width
                   safe_ped = 0;
%                    disp('collide edge')
                    break;
               end
            end
            
            end
        end
    else
        break;
    end
end