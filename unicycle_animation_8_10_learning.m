% Simulate the unicycle model 
clear; close; clc; 

% Setup time vector 
dT = 0.01;
tF = 20; 
t = 0:dT:tF; 
N = length(t); 

% Unicycle model parameters 
V = 10; 
b = 1; 

% Choose input - heading rate of change 
% u = exp(-t/10).*sin(2*pi*t/10); 
% u = 10*sin(t)/15; 
% u = sin(4*pi*t)+cos(2*pi*t/10);
u = sin(pi*t/100)+cos(pi*5*t/10);
% u = zeros(1,N); 

% setup state vectors [x; y; theta]
x = zeros(N,1); 
y = zeros(N,1); 
th = zeros(N,1); 

% Initial conditions 
x(1) = 2; 
y(1) = 2;
th(1) = 0; 

% Simulate using input 
for i = 2:N
    x(i) = x(i-1)+V*cos(th(i-1))*dT + randn/100;
    y(i) = y(i-1)+V*sin(th(i-1))*dT + randn/100;
    th(i) = th(i-1)+b*u(i)*dT + randn/100; 
    
    % Used to solve wrap-around issue.
    if th(i) > 2*pi
        th(i) = th(i) - 2*pi;
    elseif th(i) < 0
        th(i) = th(i) + 2*pi;
    end
    
    % Determine if the drone is close to the edge of the environment and
    % change route if it is.
    if  (x(i) > 95 && th(i) >= (3*pi)/2 || x(i) > 95 && th(i) <= pi/2) || ...
            x(i) < 5 && th(i) >= pi/2 && th(i) <= (3*pi)/2 || ...
            y(i) > 95 && th(i) >= 0 && th(i) <= pi || ...
            y(i) < 5 && th(i) >= pi && th(i) <= 2*pi 
        th(i) = th(i) + (3*pi)/180;
        u(i) = b*((3*pi)/180);
    end
end

drone_x = zeros(N,1); 
drone_y = zeros(N,1); 
drone_th = zeros(N,1); 
drone_u = zeros(N,1); 
theta_c = zeros(N,1); 

% Range (radius) of drone sight measured as the Euclidean distance.
range = 20;

% drone_x(1) = -1; 
% drone_y(1) = 1;
% drone_th(1) = pi/2;
drone_x(1) = 3; 
drone_y(1) = 0;
drone_th(1) = pi/2;

% Drone control parameters 
Kth = 200; 
bth = 2; 
Vth = 20; 

% Prediction Step
x_hat = zeros(3,N); 
P = [20 0 0; 0 20 0; 0 0 1];
Q = [0.5 0 0; 0 0.5 0; 0 0 0.05];
C = [1 0 0; 0 1 0; 0 0 1];
R = [2 0 0; 0 2 0; 0 0 0.2];

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
        50 40 10 5
        30 50 10 3
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

for k = 2:N
    % Prediction
    A = [0 0 -V*sin(x_hat(3,k-1)); 0 0 V*cos(x_hat(3,k-1)); 0 0 0];
    dx_hat = dT*[V*cos(x_hat(3,k-1)) V*sin(x_hat(3,k-1)) u(k)/2];
    x_hat_next = x_hat(:,k-1) + dx_hat';
    
    % Covariance 
    dP = (A*P + A'*P + Q) * dT;
    P = P + dP;
    
    x_hat(:,k) = x_hat_next;
    
    % Euclidean distance between drone and target
    distance = sqrt((drone_x(k-1)-x(k))^2 + (drone_y(k-1)-y(k))^2);

    % Only take measurements when target isn't in restricted zone and
    % within range
    if msmt_matrix(round(y(k)*10)+1,round(x(k)*10)+1) && ...
       distance <= range
        
        % Calculate the true positive rate based on the Euclidean distance.
        true_pos = 0.5-0.35*atan(distance-5);
        
        % Take a measurement if we have a true positive by comparing random
        % number to false negative rate.
        if 1%rand > 1-true_pos
            true_pos_1(k) = 1;
            % Simulate a random measurement 
            msmt = [x(k)+randn/5 y(k)+randn/5 th(k)+randn/10];
            % Correction Step
            % Kalman Gain
%             K = P*C'*(C*P*C'+R)^(-1);
            K = 0.75;
            test(k) = 1;
            % Use measurement to update the estimate x_hat and the covariance P 
            x_hat_prime = x_hat_next + K*(msmt' - C*x_hat_next);
            P_prime = P - K*C*P;
            x_hat(:,k) = x_hat_prime;
            P = P_prime;
        % The else block represents the case of a false negative.
        else
            true_pos_1(k) = -1;
        end
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

    msmt_ratio(k) = nnz(test)/(k-nnz(test));
    % Check that UAV avoids restricted area
%     while drone_x(k) < 20 && drone_x(k) > 10 && drone_y(k) > 30 && drone_y(k) < 40
%         drone_th(k) = drone_th(k) + 2;
%         drone_x(k) = drone_x(k-1)+Vth*cos(drone_th(k))*dT;
%         drone_y(k) = drone_y(k-1)+Vth*sin(drone_th(k))*dT;
%     end
end

% Setup animation 
xmin = 0;%min(drone_x)-1;%min(x)-1;
xmax = 100;%max(x)+1+2; 
ymin = 0;%min(drone_y)-1;%min(y)-1; 
ymax = 100;%max(y)+1+2; 

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
    pause(0.10);
%     pause(0.00010);
end

% Plot the ratio between observed time and unobserved time.
figure(2);
plot(t,msmt_ratio);

% Print minimum ratio number after 5 timesteps.
fprintf("Minimum value of observed/unobserved ratio: %f\n", min(msmt_ratio(5:end)));