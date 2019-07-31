state_data_matrix_10 = zeros(100,7);
state_data_matrix_20 = zeros(100,7);
state_data_matrix_40 = zeros(100,7);
for i = 1:100
    file_name_pre = 'seniorcar_state_condition2_10_';
    file_name_center = num2str(i);
    file_name = [file_name_pre,file_name_center];
    state_data_matrix_10(i,:) = generate_data_for_analysis(file_name);
end
save('condition2_10_state_data_matrix','state_data_matrix_10','-ascii');
for i = 1:100
    file_name_pre = 'seniorcar_state_condition2_20_';
    file_name_center = num2str(i);
    file_name = [file_name_pre,file_name_center];
    state_data_matrix_20(i,:) = generate_data_for_analysis(file_name);
end
save('condition2_20_state_data_matrix','state_data_matrix_20','-ascii');
for i = 1:100
    file_name_pre = 'seniorcar_state_condition2_40_';
    file_name_center = num2str(i);
    file_name = [file_name_pre,file_name_center];
    state_data_matrix_40(i,:) = generate_data_for_analysis(file_name);
end
save('condition2_40_state_data_matrix','state_data_matrix_40','-ascii');