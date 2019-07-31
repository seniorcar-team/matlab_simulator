
comfort_value_matrix_10 = zeros(100,2);
comfort_value_matrix_20 = zeros(100,2);
comfort_value_matrix_40 = zeros(100,2);
for i = 1:100
    file_name_pre = 'seniorcar_state_condition3_10';
    file_name_center = num2str(i);
    file_name_post = '_acceleration_and_jerk';
    file_name = [file_name_pre,file_name_center,file_name_post];
    file_tag = ['condition3_10_',file_name_center,'comfort_value_array'];
    comfort_value_matrix_10(i,:) = calculate_riding_comfort_indicator_ver2(file_name,file_tag);
end
save('condition3_10_comfort_value_matrix','comfort_value_matrix_10','-ascii');
for i = 1:100
    file_name_pre = 'seniorcar_state_condition3_20_';
    file_name_center = num2str(i);
    file_name_post = '_acceleration_and_jerk';
    file_name = [file_name_pre,file_name_center,file_name_post];
    file_tag = ['condition3_20_',file_name_center,'comfort_value_array'];
    comfort_value_matrix_20(i,:) = calculate_riding_comfort_indicator_ver2(file_name,file_tag);
end
save('condition3_20_comfort_value_matrix','comfort_value_matrix_20','-ascii');
for i = 1:100
    file_name_pre = 'seniorcar_state_condition3_40_';
    file_name_center = num2str(i);
    file_name_post = '_acceleration_and_jerk';
    file_name = [file_name_pre,file_name_center,file_name_post];
    file_tag = ['condition3_40_',file_name_center,'comfort_value_array'];
    comfort_value_matrix_40(i,:) = calculate_riding_comfort_indicator_ver2(file_name,file_tag);
end
save('condition3_40_comfort_value_matrix','comfort_value_matrix_40','-ascii');