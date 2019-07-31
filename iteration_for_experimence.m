Pedestrian_speed = (5:16)/10;
direction = (9:18)*10*(-1);
TTI_about = (11:25)/10;
file_num_array1 = [];
file_num_array2 = [];
file_num_array3 = [];
file_num_array4 = [];
acceleration_data_matrix = zeros(10000,4);
dd = datestr(now,'mmdd_HHMM');
for i = 1:10000
    for j = 1:5
        ps = Pedestrian_speed(randi(12));
        pd = deg2rad(direction(randi(10)));
        tti = TTI_about(randi(15));
        Pedestrian_velocity = [ ps* cos(pd),ps * sin(pd)];
        initial_r_velocity = [1.1-Pedestrian_velocity(1),-Pedestrian_velocity(2)];
        Pedestrian_position = tti * initial_r_velocity;

        Pedestrian(j) = MovingObject_ver2( Pedestrian_position, ps, pd, Pedestrian_position+Pedestrian_velocity*5);
    end
    
    file_num = i;

    acceleration_c1 = simulation_for_condition1_to_compare_acceleration(Pedestrian,file_num);
    acceleration_c2 = simulation_for_condition2_to_compare_acceleration(Pedestrian,file_num);
    acceleration_data_matrix(i,:) = [acceleration_c1,acceleration_c2];
    if acceleration_c1(1)==0
        continue;
    end
    if acceleration_c2(1)==0
        continue;
    end
    if (acceleration_c1(1) - acceleration_c2(1))>0.1
        file_num_array1 = [file_num_array1;file_num];
    end

    if (acceleration_c1(1) - acceleration_c2(1))>0.2
        file_num_array2 = [file_num_array2;file_num];
    end

    if (acceleration_c1(1) - acceleration_c2(1))>0.3
        file_num_array3 = [file_num_array3;file_num];
    end

    if (acceleration_c1(1) - acceleration_c2(1))>0.05
        file_num_array4 = [file_num_array4;file_num];
    end
    if rem(i,100) == 0
        save('acceleration_data_matrix','acceleration_data_matrix','-ascii');
        file_name = ['F_num_array1',dd];
        save(file_name,'file_num_array1','-ascii');
        file_name = ['F_num_array2',dd];
        save(file_name,'file_num_array2','-ascii');
        file_name = ['F_num_array3',dd];
        save(file_name,'file_num_array3','-ascii');
        file_name = ['F_num_array4',dd];
        save(file_name,'file_num_array4','-ascii');
    end
end
disp(length(file_num_array1));
disp(length(file_num_array2));
disp(length(file_num_array3));
disp(length(file_num_array4));
file_name = ['F_num_array1',dd];
save(file_name,'file_num_array1','-ascii');
file_name = ['F_num_array2',dd];
save(file_name,'file_num_array2','-ascii');
file_name = ['F_num_array3',dd];
save(file_name,'file_num_array3','-ascii');
file_name = ['F_num_array4',dd];
save(file_name,'file_num_array4','-ascii');
