function comfort_values = calculate_iso_comfort_indicator_complete_ver(seniorcar_state_array)
%�ԗ��i�s���������Ƃ���,���̃v���O�����͑S�̂��炠�鎞�ԋ�Ԃ�؂�o���ď��S�n�w�W�̌v�Z���s���Ă���
if size(seniorcar_state_array,1) ~= 5
    seniorcar_state_array = transpose(seniorcar_state_array);
end
L = size(seniorcar_state_array,2);%������̒���
Fs = 10;
T = 1/Fs;
t = (0:L-1)*T;
acceleration_x = (seniorcar_state_array(4,2:L)-seniorcar_state_array(4,1:L-1))./T;
acceleration_x = [acceleration_x,acceleration_x(end)];%�z�񒷂��̒���
acceleration_y = seniorcar_state_array(4,:).^2*tan(seniorcar_state_array(5,:))./0.9;%
acceleration_yaw = (seniorcar_state_array(3,2:L)-seniorcar_state_array(3,1:L-1))./T;
acceleration_yaw = [acceleration_yaw,acceleration_yaw(end)];%�z�񒷂��̒���
%{
y = fft(acceleration);
angle_intercept = angle(y);
P2 = abs(y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = Fs*(0:(L/2))/L;
[~,ind] = min(abs(f-2));
f(ind:end)=[];
P1(ind:end)=[];
angle_intercept(ind:end) = [];
%plot(f,P1);
weight = -0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443;%JIS�̕\����ΏۂƂ����ԁi�QH���ȉ��j�̒l��excel�ɑł����݁C�ߎ��Ȑ����쐬�������̂���D
array = P1.*weight;
sin_array = sin(2*pi*f.*t+angle_intercept);
comfort_value = dot(array,sin_array);
%}
L2 = 10;
comfort_value_matrix_x = zeros(1,L2);
comfort_value_matrix_y = zeros(1,L2);
comfort_value_matrix_yaw = zeros(1,L2);
comfort_value_matrix = zeros(1,L-L2);

for i = 1+L2:L
    ax_f = fft(acceleration(i-L2:i));
    angle_intercept_x = angle(ax_f);
    P2 = abs(ax_f/L2);
    P1 = P2(1:L2/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L2/2))/L2;%���g��
    [~,ind] = min(abs(f-2));
    f(ind:end)=[];
    P1(ind:end)=[];%�U��
    angle_intercept_x(ind:end) = [];
    weight_x = round(-0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443,3);%JIS�̕\����ΏۂƂ����ԁi�QH���ȉ��j�̒l��excel�ɑł����݁C�ߎ��Ȑ����쐬�������̂���D
    array_x = P1.*weight_x;%�␳�U��
    angle_intercept_x(ind:end) = [];
    
    for j = 1:L2
        sin_array = sin(2*pi*f*t(j)+angle_intercept_x);
        comfort_value_matrix_x(j) = dot(array_x,sin_array)^2*T;%�ϕ��̑�����s������,�e�����̎��g���␳�����x�̏d�ˍ��킹��2��ƒP�ʎ��Ԃ̐�
    end
    ay_f = fft(acceleration_y(i-L2:i));
    angle_intercept_y = angle(ay_f);
    P4 = abs(ay_f/L2);
    P3 = P4(1:L2/2+1);
    P3(2:end-1) = 2*P3(2:end-1);
    f = Fs*(0:(L2/2))/L2;%���g��
    [~,ind] = min(abs(f-2));
    f(ind:end)=[];
    P3(ind:end)=[];%�U��
    angle_intercept_y(ind:end) = [];
    weight_y= round(-0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443,3);%JIS�̕\����ΏۂƂ����ԁi�QH���ȉ��j�̒l��excel�ɑł����݁C�ߎ��Ȑ����쐬�������̂���D
    array_y = P3.*weight_y;%�␳�U��
    angle_intercept_y(ind:end) = [];
    for j = 1:L2
        sin_array = sin(2*pi*f*t(j)+angle_intercept_y);
        comfort_value_matrix_y(j) = dot(array_y,sin_array)^2*T;%�ϕ��̑�����s������
    end
    ayaw_f = fft(acceleration_yaw(i-L2:i));
    angle_intercept_yaw = angle(ayaw_f);
    P6 = abs(ayaw_f/L2);
    P5 = P6(1:L2/2+1);
    P5(2:end-1) = 2*P5(2:end-1);
    f = Fs*(0:(L2/2))/L2;%���g��
    [~,ind] = min(abs(f-2));
    f(ind:end)=[];
    P5(ind:end)=[];%�U��
    angle_intercept_yaw(ind:end) = [];
    weight_yaw = round(-0.0075*f.^6 + 0.125*f.^5 - 0.8276*f.^4 + 2.7502*f.^3 - 4.7986*f.^2 + 4.1317*f - 0.3633,3);
    array_yaw = P5.*weight_yaw;%�␳�U��
    angle_intercept_yaw(ind:end) = [];
    for j = 1:L2
        sin_array = sin(2*pi*f*t(j)+angle_intercept_yaw);
        comfort_value_matrix_yaw(j) = dot(array_yaw,sin_array)^2*T;%�ϕ��̑�����s������
    end
    comfort_value_matrix(i) = sqrt(sum(comfort_value_matrix_x)/(L2*T)+sum(comfort_value_matrix_y)/(L2*T)+0.2^2*sum(comfort_value_matrix_yaw)/(L2*T));%0.2�͕����{��
    
end
filename_2 = [file_tag,'_complete_ver'];
save(filename_2,'comfort_value_matrix','-ascii');
comfort_values =[max(comfort_value_matrix)];%�������̂݁A�O�㍶�E�̏�