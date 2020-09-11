clear; close; clc; 

% Setup time vector 
dT = 0.01;
tF = 20; 
t = 0:dT:tF; 
N = length(t); 

% Unicycle model parameters 
V = 10; 
b = 1; 

% Desired functions
xd = 32*cos(t/2)+50;
yd = 35*sin(t)+35;

% State Vectors
x = zeros(N,1); 
y = zeros(N,1);
th = zeros(N,1);
ux = zeros(N,1); 
uy = zeros(N,1);

% Initial conditions 
x(1) = 70; 
y(1) = 30;

kp = 2;
% Simulate using desired functions 
for i = 2:N
    ux(i) = kp*(xd(i) - x(i-1));
    uy(i) = kp*(yd(i) - y(i-1));
    x(i) = x(i-1) + ux(i)*dT;
    y(i) = y(i-1) + uy(i)*dT;
    th(i) = atan2(uy(i),ux(i));
end

drone_x = zeros(N,1); 
drone_y = zeros(N,1); 
drone_th = zeros(N,1); 
drone_u = zeros(N,1); 
theta_c = zeros(N,1);
distance = zeros(N,1); 

% Range (radius) of drone sight measured as the Euclidean distance.
range = 20;

drone_x(1) = 65; 
drone_y(1) = 35;
drone_th(1) = pi/2;

% Drone control parameters 
Kth = 200; 
bth = 5; 
Vth = 40; 

% Used for the measurement accuracy and ratio between measured time and
% unobserved time.
test = zeros(N,1);
true_pos_1 = zeros(N,1);
msmt_ratio = zeros(N,1);

% 2D matrix representing locations where measurements cannot be taken.
msmt_matrix = ones(100*10,100*10);

% Set up no-fly zone.
% Each row of the matrix represents one rectangle where the first two
% indexes represent the x and y coordinates and the last two represent the
% offset in the x and y directions. Ex: in the first row, (10,30) is the
% starting location and 5 is offset in x and 10 is offset in y.
rect = [10 30 5 10;
        20 5 10 10;
        65 55 5 5;
        55 15 6 8;
        50 40 10 5;
        30 50 10 3;
        10 90 10 5];

% Populate measurement matrix based on rectangles generated. 1 represents
% a square where the drone can measure the target, while 0 is the
% opposite.
figure(1); hold on; 
for i = 1:size(rect)
    rectangle('position',rect(i,:));
    msmt_matrix(rect(i,1)*10:rect(i,1)*10+rect(i,3)*10,...
                rect(i,2)*10:rect(i,2)*10+rect(i,4)*10) = 0;
end
msmt_matrix = msmt_matrix';

% Prediction Step
x_hat = zeros(2,N); 
x_hat(:,1) = [63; 31]; 
% theta_hat = ones(5,1);
theta_hat = zeros(5,1);
ux_hat = zeros(N,1); 
uy_hat = zeros(N,1); 
ke = 8;
gamma = eye(5,5);

for k = 2:N    
    % Approximate input u
%     ux_hat(k) = xd(k) - (xd(k)-x(k)) - getBasisFunctions(x_hat(1,k-1))*theta_hat;
%     uy_hat(k) = yd(k) - (yd(k)-y(k)) - getBasisFunctions(x_hat(2,k-1))*theta_hat;
    ux_hat(k) = (xd(k-1) - getBasisFunctions(x_hat(1,k-1)))*theta_hat;
    uy_hat(k) = (yd(k-1) - getBasisFunctions(x_hat(2,k-1)))*theta_hat;
%     ux_hat(k) = (xd(k) - getBasisFunctions(x_hat(1,k-1)))*theta_hat;
%     uy_hat(k) = (yd(k) - getBasisFunctions(x_hat(2,k-1)))*theta_hat;

    % Euclidean distance between drone and target
    distance(k) = sqrt((drone_x(k-1)-x(k))^2 + (drone_y(k-1)-y(k))^2); 
    
    % Only take measurements when target isn't in restricted zone and
    % within range
    if msmt_matrix(round(y(k)*10)+1,round(x(k)*10)+1) && (distance(k) <= range)
        
        % Take a measurement and update estimate
        true_pos_1(k) = 1;

        % Simulate a random measurement
        msmt = [x(k) y(k)]; 
%         msmt = [x(k)+randn/5 y(k)+randn/5]; 

        % Estimator using measurement.
        x_hat_dot = ux(k) + ke*(msmt(1) - x_hat(1,k-1));
        y_hat_dot = uy(k) + ke*(msmt(2) - x_hat(2,k-1));
%         x_hat_dot = ux_hat(k) + ke*(msmt(1) - x_hat(1,k-1));
%         y_hat_dot = uy_hat(k) + ke*(msmt(2) - x_hat(2,k-1));

        test(k) = 1;
        
        % Parameter update law
        theta_hat_dot = gamma*getBasisFunctions(x_hat(1,k))'*(x_hat(1,k)-msmt(1));
        theta_hat = theta_hat + theta_hat_dot*dT; 
    else
        % Predict target estimate into the future
        x_hat_dot = ux(k); %ux_hat(k); 
        y_hat_dot = uy(k); %uy_hat(k); 
    end

    % Use deriviative of x_hat/y_hat to update 
    x_hat(:,k) = x_hat(:,k-1) + [x_hat_dot; y_hat_dot]*dT; 
    
    if mod(k,20) == 0
%         x_hat(:,k)
%         ux_hat
%         theta_hat
    end
   
    % Select a control input for drone to turn towards the target 
    theta_c(k) = atan2(x_hat(2,k)-drone_y(k-1),x_hat(1,k)-drone_x(k-1));

    % Check to make sure UAV turns the shorter way
    if drone_th(k-1) - theta_c(k) > pi
        theta_c(k) = theta_c(k) + 2*pi;
    elseif drone_th(k-1) - theta_c(k) < -pi
        theta_c(k) = theta_c(k) - 2*pi;
    end
    drone_u(k) = Kth*(theta_c(k)-drone_th(k-1))*dT; 

    % Update position and orientation of drone 
    drone_th(k) = drone_th(k-1)+dT*bth*drone_u(k);
    drone_x(k) = drone_x(k-1)+Vth*cos(drone_th(k-1))*dT;
    drone_y(k) = drone_y(k-1)+Vth*sin(drone_th(k-1))*dT;

    % Ratio between measured and unmeasured time.
    msmt_ratio(k) = nnz(test)/(k-nnz(test));
end

% Setup animation 
xmin = 0;
xmax = 100;
ymin = 0;
ymax = 100;

axis([xmin xmax ymin ymax]); 
rover_pos = plot(x(1), y(1), 'o', 'MarkerSize',10); 
rover_head = plot([x(1) x(1)+0.2*cos(th(1))], [y(1) y(1)+0.2*sin(th(1))]); 

est_rover_pos = plot(x_hat(1,1), x_hat(2,1), 's', 'MarkerSize',10); 

drone_pos = plot(drone_x(1), drone_y(1), 'x', 'MarkerSize',10); 
drone_head = plot([drone_x(1) drone_x(1)+0.2*cos(drone_th(1))], ... 
                  [drone_y(1) drone_y(1)+0.2*sin(drone_th(1))]);

drone_range = plot(range*cos(1:0.001:100),range*sin(1:0.001:100));

h = text(75,95,"");
h1 = text(75,90,"");
plot(x,y,':');
% Plot (x, y) coordinates of rover over time 
for i = 2:N
    if test(i) == 1
        set(h,'String',"MEASUREMENT");
    else
        set(h,'String',"");
    end
    
    if true_pos_1(i) == 1
        set(h1,'String',"true positive",'Color','black');
    elseif true_pos_1(i) == -1
        set(h1,'String',"false negative",'Color','red');
    else
        set(h1,'String',"");
    end

    set(rover_pos, 'xdata', x(i));
    set(rover_pos, 'ydata', y(i));
    set(rover_head, 'xdata', [x(i) x(i)+5*cos(th(i))]);
    set(rover_head, 'ydata', [y(i) y(i)+5*sin(th(i))]);

    set(est_rover_pos, 'xdata', x_hat(1,i));
    set(est_rover_pos, 'ydata', x_hat(2,i));
    
    set(drone_range, 'xdata', range*cos(1:0.001:100)+drone_x(i));
    set(drone_range, 'ydata', range*sin(1:0.001:100)+drone_y(i));
    
    set(drone_pos, 'xdata', drone_x(i));
    set(drone_pos, 'ydata', drone_y(i));
    set(drone_head, 'xdata', [drone_x(i) drone_x(i)+5*cos(drone_th(i))]);
    set(drone_head, 'ydata', [drone_y(i) drone_y(i)+5*sin(drone_th(i))]);
    pause(0.02);
%     pause(0.00010);
end

% % Plot the ratio between observed time and unobserved time.
% figure(2);
% plot(t,msmt_ratio);
% 
% % Print minimum ratio number after 5 timesteps.
% fprintf("Minimum value of observed/unobserved ratio: %f\n", min(msmt_ratio(5:end)));


function Y = getBasisFunctions(value)
    Y = [value value^2 value^3 sin(value) cos(value)];
end


