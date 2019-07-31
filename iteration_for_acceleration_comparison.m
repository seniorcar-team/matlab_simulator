Pedestrian_speed = (5:16)/10;
direction = (9:18)*10*(-1);
TTI_about = (11:25)/10;
file_num_array1 = [];
file_num_array2 = [];
file_num_array3 = [];
file_num_array4 = [];

dd = datestr(now,'mmdd_HHMM');
for i = 1:12
    for j = 1:10
        for k = 1:15
    
            Pedestrian_velocity = [Pedestrian_speed(i) * cos(deg2rad(direction(j))),Pedestrian_speed(i) * sin(deg2rad(direction(j)))];
            initial_r_velocity = [1.1-Pedestrian_velocity(1),-Pedestrian_velocity(2)];
            Pedestrian_position = TTI_about(k) * initial_r_velocity;

            Pedestrian(1) = MovingObject_ver2( Pedestrian_position, Pedestrian_speed(i), direction(j), Pedestrian_position+Pedestrian_velocity*5);
            file_num = i*10000+j*100+k;
            
            max_acceleration_c1 = simulation_for_condition1_to_compare_acceleration(Pedestrian,file_num);
            max_acceleration_c2 = simulation_for_condition2_to_compare_acceleration(Pedestrian,file_num);
            
            if (max_acceleration_c1 - max_acceleration_c2)>0.1
                file_num_array1 = [file_num_array1;file_num];
            end
            
            if (max_acceleration_c1 - max_acceleration_c2)>0.2
                file_num_array2 = [file_num_array2;file_num];
            end
            
            if (max_acceleration_c1 - max_acceleration_c2)>0.3
                file_num_array3 = [file_num_array3;file_num];
            end
            
            if (max_acceleration_c1 - max_acceleration_c2)>0.05
                file_num_array4 = [file_num_array4;file_num];
            end
            

            
            
        end
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
