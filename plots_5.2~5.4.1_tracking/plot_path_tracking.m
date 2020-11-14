figure(1);
plot(Xp_ref,Yp_ref,'r');grid;
axis equal
xlabel('X (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Y (m)','FontName','Palatino Linotype','FontSize',8);
set(gca,'FontSize',8);
text(0.03,0.96,'(a)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Reference Trajectory");

figure(2);
plot(s,Kr_ref,'r');
grid;
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Curvature \kappa (1/m)','FontName','Palatino Linotype','FontSize',8);
set(gca,'FontSize',8);
text(0.03,0.96,'(b)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Reference Curvature");

figure(3);
plot(s,Vx_ref,'g-.',s,Vp1_ref,'b--',s,Vp2_ref,'r');
grid;
axis([0,300,0,50]);
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Longitudinal velocity (m/s)','FontName','Palatino Linotype','FontSize',8);
legend('Fx=0','Integrate Forward','Integrate Backward', 'Location','northeast');
set(gca,'FontSize',8);
set(gcf,'unit','centimeters','position',[10 5 9 6.75]);
title("Forward-Backward Integrated Velocity Profile");

figure(4);
plot(s,lat_disp,'b');
grid on;
axis([0,300,-90,10]);
hold on;
plot(s,Yp_ref,'r--');
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Vehicle lateral position (m)','FontName','Palatino Linotype','FontSize',8);
legend('The plant output','Reference','Location','best');
set(gca,'FontSize',8);
text(0.03,0.96,'(a)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Lateral Position Tracking");

figure(5);
plot(s,psi1,'b');
grid;
hold on;
plot(s,Psi_ref,'r--');
legend('The plant output','Reference','Location','best');
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Vehicle heading angle (rad)','FontName','Palatino Linotype','FontSize',8);
legend('The plant output','Reference','Location','best');
set(gca,'FontSize',8);
text(0.03,0.9,'(b)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Heading Angle Tracking");

figure(6);
plot(long_disp,lat_disp,'b');
hold on;
plot(Xp_ref,Yp_ref,'r--');
grid;
axis equal
xlabel('X (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Y (m)','FontName','Palatino Linotype','FontSize',8);
legend('The plant output','Reference path','Location','best');
set(gca,'FontSize',8);
text(0.03,0.96,'(c)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Path Tracking");

figure(7);
plot(s,xdot,'b'); 
grid;
hold on;
plot(s,Vxp_ref,'r--');
axis([0,300,0,50]);
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Longitudinal velocity (m/s)','FontName','Palatino Linotype','FontSize',8);
legend('The plant output','Reference','Location','best');
set(gca,'FontSize',8);
text(0.03,0.96,'(d)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Longitudinal Velocity Tracking");

figure(8);
plot(s,delta_f,'b');
grid;
axis([0,300,-0.25,0.2]);
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Steering angle (rad)','FontName','Palatino Linotype','FontSize',8);
set(gca,'FontSize',8);
text(0.03,0.96,'(a)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Steering Angle Control Input");

figure(9);
plot(s,Tra,'b',s,Trb,'g');
axis([0,300,0,4000]);
grid;
xlabel('Distance along path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Driving & braking torque (Nm)','FontName','Palatino Linotype','FontSize',8);
legend('total wheel driving torque','total wheel braking torque','Location','northeast');
set(gca,'FontSize',8);
text(0.03,0.96,'(b)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
title("Driving and Braking Control Input");