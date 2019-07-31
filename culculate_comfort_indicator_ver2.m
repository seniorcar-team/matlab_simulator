function return_value = culculate_comfort_indicator_ver2(file_name)
%file_nameにはseniorcar_stateファイルの名前を入れる．
%x,y,yaw,speed,delta
A = load(file_name);
time_num = size(A,1);
acceleration=zeros(1,time_num);
t = linspace(0,time_num-1,time_num);
for i=1:time_num
    acceleration(i) = A(i,4)^2*tan(A(i,5))/0.9;
    if isreal(acceleration(i))==false
        acceleration(i) =0;
    end
end
integral_acceleration = 0;
for i = 2:time_num
    if acceleration(i)*acceleration(i-1) >= 0%同符号
    integral_acceleration = integral_acceleration...
        +(abs(acceleration(i))+abs(acceleration(i-1)))/2*0.1;
    elseif acceleration(i)*acceleration(i-1) < 0%異符号
        integral_acceleration = integral_acceleration...
            +(abs(acceleration(i))+abs(acceleration(i-1)))/4*0.1;
    end
end
jerk = zeros(1,time_num);
for i = 1:time_num-1
    jerk(i) = abs(acceleration(i)-acceleration(i+1))/0.1;
end
integral_jerk = 0;
for i = 1:time_num-1
    integral_jerk=integral_jerk+jerk(i);
end
acceleration_and_jerk = [t;acceleration;jerk;A(1:length(t),4)'];
filename = [file_name,'_acceleration_and_jerk',];
save(filename,'acceleration_and_jerk','-ascii');



return_value = [integral_acceleration,time_num/10,integral_acceleration/(time_num/10),integral_jerk,integral_jerk/(time_num/10)];
    %台形面積で近似
