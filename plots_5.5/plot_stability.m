figure;
plot(s,beta,'b','LineWidth',0.5);%
hold on;
grid;
xlabel('Distance Along Path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Vehicle sideslip angle (rad)','FontName','Palatino Linotype','FontSize',8);
beta_upper=atan(0.02*mu*g);
for i=1:length(beta)
beta_max(i)=beta_upper;
beta_min(i)=-beta_upper;
end
hold on;
hx3=plot(s,beta_max,'r','LineWidth',0.8);
hold on;
hx4=plot(s,beta_min,'r','LineWidth',0.8);
set(hx4,'handlevisibility','off');
axis([0,260,-0.2,0.2]);
legend('The plant output',' Sideslip angle boundery','location','best');
set(gca,'FontName','Palatino Linotype','FontSize',8);
text(0.03,0.96,'(c)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);


figure;
plot(ydot,wz1,'o-b','LineWidth',0.5,'MarkerSize',2.5,'MarkerFaceColor','w');%
% set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);
hold on;
grid;
% axis([-4,4.2,-1,2.1]);
xlabel('Lateral velocity (m/s)','FontName','Palatino Linotype','FontSize',8);
ylabel('Yaw rate (rad/s)','FontName','Palatino Linotype','FontSize',8);
% plot the envelope
[max_wz1,pos_u]=max(wz1);
[min_wz1,pos_l]=min(wz1);
psi_max=yawrate_up(pos_u);
psi_min=yawrate_low(pos_l);
% psi_l=psi_min;
Vx=min(xdot);
k=0;
for i=psi_min:0.01:psi_max
    k=k+1;
    psi_x=i;
    ydot_r=afa_sat*Vx+b*psi_x;
    ydot_right(k)=ydot_r;
end
k=0;
for i=psi_min:0.01:psi_max
    k=k+1;
    psi_x1=i;
    ydot_l=-afa_sat*Vx+b*psi_x1;
    ydot_left(k)=ydot_l;
end
k=0;
for i=max(ydot_left):0.01:max(ydot_right)
    k=k+1;
    upper(k)=psi_max;
end
upper_x=max(ydot_left):0.01:max(ydot_right);
hx4=plot(upper_x,upper,'r','LineWidth',0.8);
set(hx4,'handlevisibility','off');
k=0;
for i=min(ydot_left):0.01:min(ydot_right)
    k=k+1;
    lower(k)=psi_min;
end
lower_x=min(ydot_left):0.01:min(ydot_right);
hx3=plot(lower_x,lower,'r','LineWidth',0.8);
set(hx3,'handlevisibility','off');
y_psi=psi_min:0.01:psi_max;
hx1=plot(ydot_left,y_psi,'r','LineWidth',0.8);
% set(hx1,'handlevisibility','off');
hold on;
hx2=plot(ydot_right,y_psi,'r','LineWidth',0.8);
set(hx2,'handlevisibility','off');
legend('The plot output','Yaw stability envelope','location','best');
set(gca,'FontName','Palatino Linotype','FontSize',8);
text(0.03,0.96,'(a)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);

figure;
plot(s,wz1,'b','LineWidth',0.5);
hold on;
grid;
xlabel('Distance Along Path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Yaw rate (rad/s)','FontName','Palatino Linotype','FontSize',8);
axis([0,260,-1.5,1.5]);
hw1=plot(s,[yawrate_up(1),yawrate_up],'r','LineWidth',0.8);%
hw2=plot(s,[yawrate_low(1),yawrate_low],'r','LineWidth',0.8);%
set(hw2,'handlevisibility','off');
legend('The plant output',' Yaw rate boundery','location','best');
set(gca,'FontName','Palatino Linotype','FontSize',8);
text(0.03,0.96,'(d)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);

figure;
plot(s,phi1,'b','LineWidth',0.5);%
hold on;
grid;
xlabel('Distance Along Path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Roll angle (rad)','FontName','Palatino Linotype','FontSize',8);
axis([0,260,-0.2,0.2]);
h1=plot(s,[roll_up(1),roll_up],'r','LineWidth',0.8);%
% set(h1,'handlevisibility','off');
h2=plot(s,[roll_low(1),roll_low],'r','LineWidth',0.8);%
set(h2,'handlevisibility','off');
legend('The plant output',' Roll angle boundery','location','best');
set(gca,'FontName','Palatino Linotype','FontSize',8);
text(0.03,0.96,'(b)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);

figure;
plot(s,[LTR_ratio(1),LTR_ratio],'b','LineWidth',0.5);%
hold on;
grid;
xlabel('Distance Along Path s (m)','FontName','Palatino Linotype','FontSize',8);
ylabel('Lateral load transfer ratio (LTR)','FontName','Palatino Linotype','FontSize',8);
axis([0,260,-1.2,1.2]);
hm1=plot(s,ones(1,length(s)),'r','LineWidth',0.8);%
% set(h1,'handlevisibility','off');
hm2=plot(s,-ones(1,length(s)),'r','LineWidth',0.8);%
set(hm2,'handlevisibility','off');
legend('The plant output LTR','LTR boundery','location','best');
set(gca,'FontName','Palatino Linotype','FontSize',8);
text(0.03,0.96,'(e)','Units','normalized','FontName','Palatino Linotype','FontSize',8)
set(gcf,'unit','centimeters','position',[10 5 7.34 5.5]);