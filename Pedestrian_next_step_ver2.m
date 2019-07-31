function Pedestrian_next_step = Pedestrian_next_step_ver2(Pedestrian,walking_cycle)
    Pedestrian_next_step = Pedestrian;
    ped_num = length(Pedestrian);
    for i = 1:ped_num
        Pedestrian_next_step(i).estimated_position = Pedestrian_next_step(i).tmp_position + Pedestrian_next_step(i).velocity * walking_cycle;
        
    end
end