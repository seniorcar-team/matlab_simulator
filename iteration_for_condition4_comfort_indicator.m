filename_pre = 'seniorcar_state_condition4_';
comfort_indicator_matrix_ped_num10 = zeros(30,5);
comfort_indicator_matrix_ped_num20 = zeros(30,5);
comfort_indicator_matrix_ped_num40 = zeros(30,5);
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
    B = load(filename);
    filename = [filename_pre,'condition4_',ped_num_str,'_',file_number_str];
    return_value = culculate_comfort_indicator_ver2(filename);
    
    if rem(i,3) == 1
        comfort_indicator_matrix_ped_num10(ceil(i/3),:) = return_value(1:5);
    elseif rem(i,3) == 2
        comfort_indicator_matrix_ped_num20(ceil(i/3),:) = return_value(1:5);
    else
        comfort_indicator_matrix_ped_num40(ceil(i/3),:) = return_value(1:5);
    end
end
    
file_name_for_comfort_indicator = 'condition4_comfort_indicator_10';
save(file_name_for_comfort_indicator,'comfort_indicator_matrix_ped_num10','-ascii');

file_name_for_comfort_indicator = 'condition4_comfort_indicator_20';
save(file_name_for_comfort_indicator,'comfort_indicator_matrix_ped_num20','-ascii');

file_name_for_comfort_indicator = 'condition4_comfort_indicator_40';
save(file_name_for_comfort_indicator,'comfort_indicator_matrix_ped_num40','-ascii');