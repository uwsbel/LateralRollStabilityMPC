% Load Saved Data
load("plots_5.1_verification/step_2814.mat");
load("plots_5.1_verification/step-carsim.mat");

% Display Figures
t1=0:Tsim;

figure(1)
plot(t1*0.001,states.signals.values(:,6)*pi/180,'b:','LineWidth',1.2);
hold on;
plot(t2*0.001,yaw_rate_14,'r');%14dof
plot(t1*0.001,yaw_rate_8,'g-.');%8dof
% plot(t1*0.001,y(:,4),'r--');%bicycle
grid;
xlabel('Time (s)','FontName','Palatino Linotype','FontSize',8);
ylabel('Yaw rate (rad/s)','FontName','Palatino Linotype','FontSize',8);
legend('Carsim model','14-DOF model','8-DOF model','Location','east');
set(gca,'FontSize',8);
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Yaw Rate Step Response");

figure(2)
plot(t1*0.001,states.signals.values(:,12)*9.8,'b:','LineWidth',1.2);
hold on;
plot(t2*0.001,ay1_14,'r');
plot(t1*0.001,ay_8,'g-.');
% plot(t1*0.001,lateral_acc,'r--');
grid;
xlabel('Time (s)','FontName','Palatino Linotype','FontSize',8);
ylabel('Lateral acceleration (m/s^2)','FontName','Palatino Linotype','FontSize',8);
legend('Carsim model','14-DOF model','8-DOF model', 'Location','east');
set(gca,'FontSize',8);
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Lateral Acceleration Step Response");

figure(3)
plot(t1*0.001,states.signals.values(:,5)/3.6,'b:','LineWidth',1.2);
hold on;
plot(t2*0.001,lat_vel_14,'r');
plot(t1*0.001,lat_vel_8,'g-.');
% plot(t1*0.001,y(:,1),'r--');
grid;
xlabel('Time (s)','FontName','Palatino Linotype','FontSize',8);
ylabel('Lateral velocity (m/s)','FontName','Palatino Linotype','FontSize',8);
legend('Carsim','14-DOF model','8-DOF model','Location','east');
set(gca,'FontSize',8);
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Lateral Velocity Step Response");

figure(4)
plot(t1*0.001,states.signals.values(:,31)*pi/180,'b:','LineWidth',1.2);
hold on;
plot(t2*0.001,roll_angle_14,'r');% 14dof
hold on;
plot(t1*0.001,roll_angle_8,'g-.');% 8dof
grid;
xlabel('Time (s)','FontName','Palatino Linotype','FontSize',8);
ylabel('Roll angle(rad)','FontName','Palatino Linotype','FontSize',8);
legend('Carsim model','14-DOF model','8-DOF model','Location','east');
set(gca,'FontSize',8);
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Roll Angle Step Response");