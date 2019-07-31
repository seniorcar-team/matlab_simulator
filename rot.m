function new_position = rot(x,theta)
A = [cos(theta) -sin(theta);sin(theta) cos(theta)];
x_vector = [x(1);x(2)];
new_position = A*x_vector;