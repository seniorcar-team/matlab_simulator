%% 歩行者との衝突判定
function collide_safe_ped = Judgement_collision(x, Pedestrian)
% 歩行者との衝突判定　1 or 0 (1 = safe, 0= collide)
global vehicle_width;
global vehicle_front_length;
global vehicle_rear_length;
global pedestrian_width;

carbody_edge = [vehicle_front_length -vehicle_width/2;
                vehicle_front_length vehicle_width/2;
               -vehicle_rear_length -vehicle_width/2;
               -vehicle_rear_length vehicle_width/2];

collide_safe_ped =1;%1がぶつからないとき
        for j = 1:length(Pedestrian)
            % relative_position
            relative_position(1) = Pedestrian(j).tmp_position(1) - x(1);
            relative_position(2) = Pedestrian(j).tmp_position(2) - x(2);
            % rotation
            relative_position_rota(1) =  relative_position(1)*cos(x(3)) + relative_position(2)*sin(x(3));
            relative_position_rota(2) =  -relative_position(1)*sin(x(3)) + relative_position(2)*cos(x(3));
            % judgement            
            % 歩行者が車両のボデーの中にいれば衝突と判定
            if relative_position_rota(1) < vehicle_front_length  + pedestrian_width/2 ...
                    && relative_position_rota(1) > - vehicle_rear_length  - pedestrian_width/2
                if relative_position_rota(2) < vehicle_width/2 && relative_position_rota(2) > - vehicle_width/2 
                    collide_safe_ped = 0;
%                     disp('collide front')               
                    break;
                end
            end
            
            if relative_position_rota(1) < vehicle_front_length && relative_position_rota(1) > - vehicle_rear_length 
                if relative_position_rota(2) < vehicle_width/2 + pedestrian_width/2 &&...
                        relative_position_rota(2) > - vehicle_width/2 - pedestrian_width/2
                    collide_safe_ped = 0;
%                     disp('collide side')                 
                    break;
                end
            end
            

            for i=1:length(carbody_edge(1,:))
               if norm(relative_position_rota-carbody_edge(i,:)) < pedestrian_width/2
                   collide_safe_ped = 0;
%                    disp('collide edge')
                    break;
               end
            end
        end