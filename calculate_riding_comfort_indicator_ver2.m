function comfort_values = calculate_riding_comfort_indicator_ver2(file_name,file_tag)

x = load(file_name);
acceleration = x(2,:);
L = length(acceleration);
Fs = 10;
T = 1/Fs;
t = (0:L-1)*T;
acceleration_2 = (x(4,2:L)-x(4,1:L-1))./T;
acceleration_2 = [acceleration_2,acceleration_2(end)];

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
weight = -0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443;%JISの表から対象とする区間（２Hｚ以下）の値をexcelに打ち込み，近似曲線を作成したものから．
array = P1.*weight;
sin_array = sin(2*pi*f.*t+angle_intercept);
comfort_value = dot(array,sin_array);
%}
L2 = 10;
comfort_value_matrix_1 = zeros(1,L2);
comfort_value_matrix_2 = zeros(1,L2);
comfort_value_matrix = zeros(1,L-L2);

for i = 1+L2:L
    y = fft(acceleration(i-L2:i));
    angle_intercept = angle(y);
    P2 = abs(y/L2);
    P1 = P2(1:L2/2+1);
    P1(2:end-1) = 2*P1(2:end-1);
    f = Fs*(0:(L2/2))/L2;%周波数
    [~,ind] = min(abs(f-2));
    f(ind:end)=[];
    P1(ind:end)=[];%振幅
    angle_intercept(ind:end) = [];
    weight = -0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443;%JISの表から対象とする区間（２Hｚ以下）の値をexcelに打ち込み，近似曲線を作成したものから．
    array = P1.*weight;%補正振幅
    angle_intercept(ind:end) = [];
    
    for j = 1:L2
        sin_array = sin(2*pi*f*t(j)+angle_intercept);
        comfort_value_matrix_1(j) = dot(array,sin_array)^2*T;%積分の操作を行うため
    end
    comfort_value_matrix_3(i) = sqrt(sum(comfort_value_matrix_1)/(L2*T));
    z = fft(acceleration_2(i-L2:i));
    angle_intercept_2 = angle(z);
    P4 = abs(z/L2);
    P3 = P4(1:L2/2+1);
    P3(2:end-1) = 2*P3(2:end-1);
    f = Fs*(0:(L2/2))/L2;%周波数
    [~,ind] = min(abs(f-2));
    f(ind:end)=[];
    P3(ind:end)=[];%振幅
    angle_intercept_2(ind:end) = [];
    weight = -0.2694*f.^4 + 1.6882*f.^3 - 3.9354*f.^2 + 3.8922*f - 0.3443;%JISの表から対象とする区間（２Hｚ以下）の値をexcelに打ち込み，近似曲線を作成したものから．
    array_2 = P3.*weight;%補正振幅
    angle_intercept_2(ind:end) = [];
    for j = 1:L2
        sin_array = sin(2*pi*f*t(j)+angle_intercept_2);
        comfort_value_matrix_2(j) = dot(array_2,sin_array)^2*T;%積分の操作を行うため
    end
    comfort_value_matrix(i) = sqrt(sum(comfort_value_matrix_1)/(L2*T)+sum(comfort_value_matrix_2)/(L2*T));
    
end
filename_2 = [file_tag,'_ver2'];
save(filename_2,'comfort_value_matrix','-ascii');
comfort_values =[max(comfort_value_matrix_3), max(comfort_value_matrix)];%横向きのみ、前後左右の順