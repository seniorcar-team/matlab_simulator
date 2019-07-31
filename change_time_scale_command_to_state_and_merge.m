function change_time_scale_command_to_state_and_merge(state_file,command_file)
%dt = 0.01
command = load(command_file);
state = load(state_file);

%fixed_command = zeros(length(command(:,3))*10,2);
x_command = [0,0,0,0,0];
x_state = [0,0,0,0,0];
command = command(command(:,2)==127,:);
command_index_over_2km = find(command(:,3) > 2);
command_first_index_over_2km = command_index_over_2km(1);
command(1:command_first_index_over_2km,3) = 2/command_first_index_over_2km*[1:command_first_index_over_2km]';
X_command = zeros(length(command(:,3))*10,5);
fixed_u_command = zeros(length(command(:,3))*10,2);
X_state = zeros(length(state(:,4)),5);
u_state = [state(:,5),deg2rad(state(:,4))];
u_command = [command(:,3)/3.6,deg2rad(command(:,4))];

for i = 1:length(command(:,3))
    for j = 1:10
        if i == 1
            fixed_u_command((i-1)*10+j,:) = [u_command(i,:)*j/10];
        else
            fixed_u_command((i-1)*10+j,:) = [u_command(i-1,:)+(u_command(i,:)-u_command(i-1,:)).*j./10]; 
        end
    end
end

for i = 1:length(command(:,3))*10
x_command = f_sim(x_command,fixed_u_command(i,:),0.01);
X_command(i+1,:) = x_command;
end
hold on;
plot(X_command(:,1),X_command(:,2),'ob');

for i = 1:length(state(:,3))
x_state = f_sim(x_state,u_state(i,:),0.01);
X_state(i+1,:) = x_state;
end
plot(X_state(:,1),X_state(:,2),'or');

end
