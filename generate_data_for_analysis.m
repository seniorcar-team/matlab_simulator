function state_array = generate_data_for_analysis(filename)
state_array = zeros(1,7);
A = load(filename);

%seniorcar_state‚Ìƒtƒ@ƒCƒ‹‚ğ“Ç‚İ‚Ş

driving_state = A(:,6);
velocity = A(:,4);


state_array(1)=length(find(round(driving_state) == 1));
state_array(2)=length(find(round(driving_state) == 2));
state_array(3)=length(find(round(driving_state) == 3));
state_array(4)=length(find(round(driving_state) == 4));
state_array(5)=length(find(round(driving_state) == 5));

time = size(A,1);
stop_time_counter=0;
collision_counter = 0;
for i = 1:time-1
    if A(i+1,7) - A(i,7) == -1
        collision_counter = collision_counter + 1;
    end
end
for j = 1:time-1
    if isinf(1/velocity(j))-isinf(1/velocity(j+1)) == 1
        stop_time_counter=stop_time_counter+1;
    end
end
state_array(6)=collision_counter;
state_array(7)=stop_time_counter;