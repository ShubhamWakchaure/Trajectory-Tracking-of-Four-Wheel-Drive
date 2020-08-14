%% ========= Wheeled Mobile Robot Project ======= %%
%  Shubham Balasaheb Wakchaure
%  131701026
% This is the animation of four wheel non steerable 
% animation for heart shaped trajectory

%% ======== Trajectory tracking of four wheel drive ====== %%

close all;clear all;clc;

%% =========== Geomtery of robot ========== %%
d = 2.5; % Longitudinal wheel centre distance
l = 1.5; %  Transverse Wheel Centre distance
a = 1; % Wheel radius
 F = VideoWriter('Straight_Line_Trajectory.avi'); F.FrameRate = 10; open(F);

%% ========== simulation parameters ======%%
dt = 0.1; % step size
tf = 200; % simulation time for one segment
t =0:dt:tf; % time span for one segment

%% =========Trajectory ================= %% 
% % 1st Trajectory is of heart shape 
% eta(:,1) = [0;5;1.5708]; % Intial condition
% th  = 0:0.01:2*pi;
% x_d = 16*(sin(th)).^3;
% y_d = 13*cos(th)-5*cos(2*th)-2*cos(3*th)-cos(4*th);
% psi_d = atan2(y_d,x_d);
% xdot_d = 48*cos(th).*(sin(th)).^2;
% ydot_d = -13*sin(th)+10*sin(2*th)+6*sin(3*th)+4*sin(4*th);
% psidot_d = atan2(ydot_d,xdot_d);
% 
% eta_d = [x_d;y_d;psi_d];

% %2nd trajectory is of circle
% eta(:,1) = [0;0;0];  % Intial condition
% th = 0:0.01:2*pi;
% x_d = 10*cos(th);
% y_d = 10*sin(th);
% psi_d = atan2(y_d,x_d);
% xdot_d = -10*sin(th);
% ydot_d = 10*cos(th);
% psidot_d = atan2(ydot_d,xdot_d);
% 
% eta_d = [x_d;y_d;psi_d];

% %3rd trajectory straight line
% eta(:,1) = [0;0;0];  % Intial condition
% x_d = 0:0.1:15;
% y_d = 2*x_d+5;
% psi_d = atan2(y_d,x_d);
% xdot_d(1:length(x_d)) = 0;
% ydot_d(1:length(y_d)) = 2;
% psidot_d = atan2(ydot_d,xdot_d);
% 
% eta_d = [x_d;y_d;psi_d];

% % 4th trajectory is straight line y = 15
% eta(:,1) = [0;0;0];  % Intial condition
% x_d = 0:0.1:15;
% y_d(1:length(x_d)) = 15;
% psi_d(1:length(x_d)) = 0;
% xdot_d(1:length(x_d)) = 0;
% ydot_d(1:length(x_d)) = 0;
% psidot_d(1:length(x_d)) = 0;
% eta_d = [x_d;y_d;psi_d];

% 5th trajectry is triafolium
eta(:,1) = [0;0;0];
x_d =  10*2*cos(2.*t).*cos(6.*t);
y_d = 10*2*cos(6.*t).*sin(2.*t);
psi_d = atan2(y_d,x_d);
xdot_d =- 12*cos(2.*t).*sin(6.*t) - 4*cos(6.*t).*sin(2.*t); 
ydot_d = - 12*cos(2.*t).*sin(6.*t) - 4*cos(6.*t).*sin(2.*t);
psidot_d = atan2(ydot_d,xdot_d);
eta_d = [x_d;y_d;psi_d];

%% =======Preliminary Calculation =====%%
k = 1; 
k_p = 1; % Proportional gain   for trajectory 3 and 4
%k_p = 15;%proportional gain for trajecory 1 and 2
for i = 1:length(x_d)
   x{:,i} = eta(1,k);%Actual X position of robot
   y{:,i} = eta(2,k);%Actual y position of robot
   psiii{:,i} = eta(3,k); %angular position of robot
   J_psi{:,i} = [ cos(psiii{:,i}) -sin(psiii{:,i}) 0;  % Jacobian Matrix
                 sin(psiii{:,i}) cos(psiii{:,i}) 0;
                               0        0        1];
   
   inv_W = (1/a)*[1 0 -d; % inverse of wheel configuration matrix
              1 0 -d;
              1 0  d;
              1 0  d];
         
    
    zeta(:,i) = inv(J_psi{:,i})*( [xdot_d(i) ; ydot_d(i) ; psidot_d(i)] + k_p*[ x_d(i)-x{:,i} ; y_d(i)-y{:,i} ; psi_d(i)-psiii{:,i} ] );
    w(:,k) = inv_W*zeta(:,i);% Wheel angular velocity matrix[w1,w2,w3,w4]
    eta_dot(:,k) = J_psi{:,i}*zeta(:,i);
    eta(:,k+1) = eta(:,k) + eta_dot(:,k) *dt;
    error(:,k) = eta_d(:,k) - eta(:,k);
    tk(k) = k*dt;
    k = k+1;
    
end

Robot =  0.5*[2.5,2.5,-2.5,-2.5,2.5;-2,2,2,-2,-2];
%% ========= Plotting The Robot tracking  ============= %%
figure
for i = 1:5:k-1
    psiii = eta(3,i); x = eta(1,i); y = eta(2,i); 
    R = [cos(psiii),-sin(psiii);
         sin(psiii),cos(psiii)];
    bot = R *Robot;
    set(gcf,'Position',[0 0 1800 1800])
    
    %% Actual robot motion plot
    subplot(2,2,1) 
    min1 = min(min(eta(1,:)),min(eta(2,:)));
    max1 = max(max(eta(1,:)),max(eta(2,:)));
    fill(bot(1,:) + x ,bot(2,:) + y,'b','LineWidth',2)
    hold on
    p2 =  plot(eta_d(1,1:i),eta_d(2,1:i),'m','LineWidth',2);
    p1 =  plot(eta(1,1:i),eta(2,1:i),'r','LineWidth',2);
    legend([p1;p2],'Actual Robot Path','Desired Robot path')
    axis([min1-5 max1+5 min1-5 max1+5]);
    title('Robot trajectory motion')
    axis square
    xlabel('x,[m]');
    ylabel('y,[m]');
    grid on
    hold off
    
    %% Plotting the angular velocities
    subplot(2,2,2)
    plot(tk(1:i),w(:,1:i),'LineWidth',2)
    min2 = min([min(w(1,:)),min(w(2,:)),min(w(3,:)),min(w(4,:))]);
    max2 = max([max(w(1,:)),max(w(2,:)),max(w(3,:)),max(w(4,:))]);
    hold on
    plot([tk(i) tk(i)],[-200 200], 'k','LineWidth',2.5)
    %axis([min2-5 max2+5 min2-5 max2+5]);
    title('Wheel angular velocity')
    legend('\omega_1','\omega_2','\omega_3','\omega_4');
    grid on
    xlabel('time (t),[s]');
    ylabel('Angular Velocity');
    hold off
    
    %% Plotting the robots pose x,y,psi
    subplot(2,2,3)
    min3 = min([min(eta(1,:)),min(eta(2,:)),min(eta(3,:))]);
    max3 = max([max(eta(1,:)),max(eta(2,:)),max(eta(3,:))]);
    plot (tk(1:i),eta(:,1:i),'LineWidth',2)
    hold on
    plot([tk(i) tk(i)],[-20 20], 'k','LineWidth',2.5)
    %axis([min3-5 max3+5 min3-5 max3+5]);
    legend('x,[m]','y,[m]','\psi,[rad]');
    title('Robot pose')
    xlabel('time (t), [s]');
    ylabel('\eta,[units]');
    grid on
    hold off
    
    %% Plotting the error between the actual and desired path
    subplot(2,2,4)
    min4 = min([min(error(1,:)),min(error(2,:)),min(error(3,:))]);
    max4 = max([max(error(1,:)),max(error(2,:)),max(error(3,:))]);
    plot (tk(1:i),error(:,1:i),'LineWidth',2)
    hold on
    plot([tk(i) tk(i)],[-20 20], 'k','LineWidth',2.5)
    %axis([min4-5 max4+5 min4-5 max4+5]);
    legend('X Error','Y Error','\psi Error')
    title('Error in x , y and \psi')
    xlabel('time (t), [s]');
    ylabel('Error');
    grid on
    hold off
    
    pause(0.01)
    writeVideo(F, getframe(figure(1)) );
    
end
 close(F); %implay('Straight_Line_Trajectory.avi');

 
