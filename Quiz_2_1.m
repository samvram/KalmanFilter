clear all;
X_ini = [30000*0.3048; 0];
% X = [y_vertical x_horizontal]

V_ini = [0; 500*5/18];
% X_dot = [y_vertical_dot x_horizontal_dot]

a = [-9.8; 0];
t_step = 1;
C = [1 0; 0 1];
A = C;
V = V_ini;
X = X_ini;
y(1) = X(1);
x(1) = X(2);
i=2;

figure;
while X(1)>0
    V = C*V + a*t_step;
    X = A*X + V*t_step;
    y(i) = X(1);
    x(i) = X(2);
    plot(x(i),y(i),'*');
    title('Real Time Missile');
    xlim([0 6000]);
    ylim([0 10000]);
    i = i+1;
    pause(0.1);
end

figure;
plot(x,y);
title('Trajectory of the Missile');
ylim([0 10000]);
xlabel('X(meters)');
ylabel('Y(meters)');
hold on;

clear all;
X_ini = [30000*0.3048; 0];
% X = [y_vertical x_horizontal]

V_ini = [0; 500*5/18];
% X_dot = [y_vertical_dot x_horizontal_dot]
k=[-0.01 0; 0 -0.01];

a_ini = [-9.8; 0];

t_step = 0.1;
A = [1 0; 0 1];
V = V_ini;
X = X_ini;
y_1(1) = X(1);
x_1(1) = X(2);
i=2;


% figure;
while X(1)>0
    V = A*V + [a_ini+k*V]*t_step;
    X = A*X + V*t_step;
    y_1(i) = X(1);
    x_1(i) = X(2);
%     plot(x(i),y(i),'*');
%     title('Real Time Missile');
%     xlim([0 2.5e7]);
%     ylim([0 10000]);
    i = i+1;
end

% figure;
plot(x_1,y_1);
% title('Trajectory of the Missile');
% ylim([0 10000]);
% xlabel('X(meters)');
% ylabel('Y(meters)');
legend('Without Drag','With Drag');


% Defining Model
clear all;
rho = 2;
ro = 3;
t_step = 0.1;
x_hat = [30000*0.308;
     0; 
     0; 
     500*5/18];
 
x = x_hat;
 
Q = [rho^2 0 0 0;
     0 rho^2 0 0;
     0 0 rho^2 0;
     0 0 0 rho^2];

R = [ro^2 0 0 0;
     0 ro^2 0 0;
     0 0 ro^2 0;
     0 0 0 ro^2];

 
P = [1 0 0 0; 
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
     
% x = [height;
%      distance;
%      velocity_y;
%      velocity_x;]

A = [1 0 t_step 0;
     0 1 0 t_step;
     0 0 1 0;
     0 0 0 1];

B = [0.5*t_step*t_step;
     0;
     t_step;
     0];
 
C = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];

I = [1 0 0 0;
     0 1 0 0;
     0 0 1 0;
     0 0 0 1];
 

 i=1;
 
 while x(1)>0
     x_actual(i,:) = x;
     x = A*x + B.*(-9.8) + rho*randn(4,1);
     y = C*x + ro*randn(4,1);
     Y(i,:) = y;
     x_hat = A*x_hat + B*-9.8;
     P = A*P*A' + Q;
     K = P*C'*(C*P*C'+R)^-1;
     x_hat = x_hat + K*(y-C*x_hat);
     x_observed(i,:) = x_hat; 
     P = (I-K*C)*P*(I-K*C)' + K*R*K';
     i = i+1;
 end
 

 figure;
 plot(x_observed(:,2), x_observed(:,1));
 hold on;
 plot(x_actual(:,2), x_actual(:,1));
 plot(Y(:,2), Y(:,1));
 legend('Estimated','Actual','Measured');
 title('Plots of the Graphs');
 xlabel('x->');
 ylabel('y->');
 
 figure;
 subplot(2,2,1);
 plot((x_actual(:,2)-x_observed(:,2)));
 title('Estimated - Actual for x');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,2);
 plot((x_actual(:,1)-x_observed(:,1)));
 title('Estimated - Actual for y');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,3);
 plot((x_actual(:,4)-x_observed(:,4)));
 title('Estimated - Actual for x_velocity');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,4);
 plot((x_actual(:,3)-x_observed(:,3)));
 title('Estimated - Actual for y_velocity');
 ylabel('Error');
 xlabel('Time');
 
 % The code for Drag case
clear all;
rho = 2;
ro = 3;
t_step = 0.1;
x_hat = [30000*0.308;
     0; 
     0; 
     500*5/18;
     -9.8;
     0];
 
x = x_hat;
k_v = -0.01;
k_h = -0.01;
 
Q = [rho^2 0 0 0 0 0;
     0 rho^2 0 0 0 0;
     0 0 rho^2 0 0 0;
     0 0 0 rho^2 0 0;
     0 0 0 0 rho^2 0;
     0 0 0 0 0 rho^2];
     
R = [ro^2 0 0 0 0 0;
     0 ro^2 0 0 0 0;
     0 0 ro^2 0 0 0;
     0 0 0 ro^2 0 0;
     0 0 0 0 ro^2 0;
     0 0 0 0 0 ro^2];

 
P = [1 0 0 0 0 0; 
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
     
% x = [height;
%      distance;
%      velocity_y;
%      velocity_x;]

A = [1 0 t_step 0 0.5*t_step^2 0;
     0 1 0 t_step 0 0.5*t_step^2;
     0 0 1 0 t_step 0;
     0 0 0 1 0 t_step;
     0 0 k_v 0 0 0;
     0 0 0 k_h 0 0];

B = [0;
     0;
     0;
     0;
     1;
     0];
 
C = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];

I = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];
 

 i=1;
 
 while x(1)>0
     x_actual(i,:) = x;
     x = A*x + B.*(-9.8) + rho*randn(6,1);
     y = C*x + ro*randn(6,1);
     Y(i,:) = y;
     x_hat = A*x_hat + B*-9.8;
     P = A*P*A' + Q;
     K = P*C'*(C*P*C'+R)^-1;
     x_hat = x_hat + K*(y-C*x_hat);
     x_observed(i,:) = x_hat; 
     P = (I-K*C)*P*(I-K*C)' + K*R*K';
     i = i+1;
 end
 

 figure;
 plot(x_observed(:,2), x_observed(:,1));
 hold on;
 plot(x_actual(:,2), x_actual(:,1));
 plot(Y(:,2), Y(:,1));
 legend('Estimated','Actual','Measured');
 title('Plots of the Graphs_DRAG');
 xlabel('x->');
 ylabel('y->');
 
 figure;
 subplot(2,2,1);
 plot((x_actual(:,2)-x_observed(:,2)));
 title('Estimated - Actual for x _Drag');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,2);
 plot((x_actual(:,1)-x_observed(:,1)));
 title('Estimated - Actual for y _Drag');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,3);
 plot((x_actual(:,4)-x_observed(:,4)));
 title('Estimated - Actual for x_velocity _Drag');
 ylabel('Error');
 xlabel('Time');
 
 subplot(2,2,4);
 plot((x_actual(:,3)-x_observed(:,3)));
 title('Estimated - Actual for y_velocity _Drag');
 ylabel('Error');
 xlabel('Time');
 