% Define the two points
x = [1, 5];
y = [2, 8];

% Compute the midpoint and radius of the circle
mid_x = (x(1) + x(2)) / 2;
mid_y = (y(1) + y(2)) / 2;
r = norm([x(1)-x(2), y(1)-y(2)]) / 2;

% Define the angle range for the half-circle
theta = linspace(-pi/2, pi/2, 100);

% Compute the x and y coordinates of the half-circle
xx = mid_x + r*cos(theta);
yy = mid_y + r*sin(theta);

% Plot the half-circle
plot(xx, yy, 'k-', 'LineWidth', 2);

% Add an arrowhead at one end
hold on;
head_length = 0.2; % adjust as needed
head_angle = pi/6; % adjust as needed
dx = xx(end)-xx(end-1);
dy = yy(end)-yy(end-1);
theta = atan2(dy, dx);
x_head = xx(end) - head_length*r*cos(theta-head_angle):...
         head_length*r*cos(theta):...
         xx(end) - head_length*r*cos(theta+head_angle);
y_head = yy(end) - head_length*r*sin(theta-head_angle):...
         head_length*r*sin(theta):...
         yy(end) - head_length*r*sin(theta+head_angle);
patch(x_head, y_head, 'k', 'LineWidth', 2);

% Add labels and formatting
axis equal;
xlabel('X');
ylabel('Y');
title('Half-Circle with Arrowhead');
grid on;
