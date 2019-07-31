filename_pre = 'seniorcar_state_condition4_';
speed_matrix_10 = zeros(100,4);
speed_matrix_20 = zeros(100,4);
speed_matrix_40 = zeros(100,4);
speed_at_a_collision_array_10 = [];
speed_at_a_collision_array_20 = [];
speed_at_a_collision_array_40 = [];
for i = 1:300
    
    file_number_str = num2str(ceil(i/3));
    if rem(i,3) == 1
        ped_num_str = num2str(10);
    elseif rem(i,3) == 2
        ped_num_str = num2str(20);
    else
        ped_num_str = num2str(40);
    end
    filename = [filename_pre,ped_num_str,'_',file_number_str];
    A = load(filename);
    B = A(:,4);
    C = A(:,7);
    C_ = [1;C];
    C_(end) = [];
    D = C-C_;
    speed_at_a_collision = [];
    is_initial_state_make_collision_occured = 0;
    for j = 1:length(D)-1
        if D(j) == -1
            speed_at_a_collision = [speed_at_a_collision;B(j)];
            if rem(i,3) == 1
                speed_at_a_collision_array_10 = [speed_at_a_collision_array_10;B(j)];
            elseif rem(i,3) == 2
                speed_at_a_collision_array_20 = [speed_at_a_collision_array_20;B(j)];
            else
                speed_at_a_collision_array_40 = [speed_at_a_collision_array_40;B(j)];
            end
            if j == 1
                is_initial_state_make_collision_occured = 1;
            end
        end
    end
    
    
    
    if rem(i,3) == 1
        speed_matrix_10(ceil(i/3),:) = [length(speed_at_a_collision),mean(speed_at_a_collision),std(speed_at_a_collision),is_initial_state_make_collision_occured];
    elseif rem(i,3) == 2
        speed_matrix_20(ceil(i/3),:) = [length(speed_at_a_collision),mean(speed_at_a_collision),std(speed_at_a_collision),is_initial_state_make_collision_occured];
    else
        speed_matrix_40(ceil(i/3),:) = [length(speed_at_a_collision),mean(speed_at_a_collision),std(speed_at_a_collision),is_initial_state_make_collision_occured];
    end
end
    
file_name_for_comfort_indicator = 'condition4_speed_matrix_at_a_collision_10';
save(file_name_for_comfort_indicator,'speed_matrix_10','-ascii');

file_name_for_comfort_indicator = 'condition4_speed_matrix_at_a_collision_20';
save(file_name_for_comfort_indicator,'speed_matrix_20','-ascii');

file_name_for_comfort_indicator = 'condition4_speed_matrix_at_a_collision_40';
save(file_name_for_comfort_indicator,'speed_matrix_40','-ascii');
mean_and_std_speed_matrix = [length(speed_at_a_collision_array_10),mean(speed_at_a_collision_array_10),std(speed_at_a_collision_array_10); ... 
    length(speed_at_a_collision_array_20),mean(speed_at_a_collision_array_20),std(speed_at_a_collision_array_20); ... 
    length(speed_at_a_collision_array_40),mean(speed_at_a_collision_array_40),std(speed_at_a_collision_array_40)];
file_name_for_comfort_indicator ='mean_and_std_speed_matrix_condition4';
save(file_name_for_comfort_indicator,'mean_and_std_speed_matrix','-ascii');