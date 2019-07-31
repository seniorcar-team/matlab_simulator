function [] = iteration_simulation_for_DWA()
PARAM = [0.15,0.25,0.1,0,0,2.0];
PARAM_4 = [0.4,0.1,0.3,0.2,0.25]*(-1);
PARAM_5 = [0.1,0.4,0.2,0.3,0.25]*(-1);
filename_pre = 'batu_pedestrian_first_state_cruise';
for a = 1:5
    PARAM = [0.15,0.25,0.1,PARAM_4(a),PARAM_5(a),2.0];
    emergency_counter_array = zeros(6,100);


    for i = 1:300
        clear Pedestrian;
        file_number_str = num2str(ceil(i/3));
        if rem(i,3) == 1
            ped_num_str = num2str(10);
        elseif rem(i,3) == 2
            ped_num_str = num2str(20);
        else
            ped_num_str = num2str(40);
        end
        filename = [filename_pre,'_',ped_num_str,'_',file_number_str];
        B = load(filename);
        ped_num = size(B,1);
        for j = 1:ped_num
            Pedestrian(j) = MovingObject_ver2( [B(j,1),B(j,2)], B(j,3)*3.6, B(j,4), [B(j,5),B(j,6)]);
            Pedestrian(j).walking_cycle = B(j,7);
            %{
            Pedestrian(j).position(1) = B(j,1);
            Pedestrian(j).position(2) = B(j,2);
            Pedestrian(j).speed = B(j,3);
            Pedestrian(j).direction = B(j,4);
            Pedestrian(j).target_position(1) = B(j,5);
            Pedestrian(j).target_position(2) = B(j,6);
            Pedestrian(j).velocity = [B(j,3)*cos(deg2rad(B(j,4))),B(j,3)*sin(deg2rad(B(j,4)))];
            Pedestrian(j).tmp_position(1) = B(j,1);
            Pedestrian(j).tmp_position(2) = B(j,2);
            Pedestrian(j).estimated_position(1) = B(j,1);
            Pedestrian(j).estimated_position(2) = B(j,2);
            Pedestrian(j).walking_cycle = rand(i);
            %}
        end
        length(Pedestrian)
        file_tag = [ped_num_str,'_',file_number_str];

        if rem(i,3) == 1
            emergency_array = simulation_for_DWA(Pedestrian,file_tag,PARAM);
            emergency_counter_array(1,ceil(i/3)) = emergency_array(1);
            emergency_counter_array(2,ceil(i/3)) = emergency_array(2);

        elseif rem(i,3) == 2
            emergency_array = simulation_for_DWA(Pedestrian,file_tag,PARAM);
            emergency_counter_array(3,ceil(i/3)) = emergency_array(1);
            emergency_counter_array(4,ceil(i/3)) = emergency_array(2);
        else
            emergency_array = simulation_for_DWA(Pedestrian,file_tag,PARAM);
            emergency_counter_array(5,ceil(i/3)) = emergency_array(1);
            emergency_counter_array(6,ceil(i/3)) = emergency_array(2);
        end
    end

    FILENAME = ['emergency_counter_array_DWA_',num2str(10*PARAM(4)),'_',num2str(10*PARAM(5))];
    save(FILENAME,'emergency_counter_array','-ascii')
end