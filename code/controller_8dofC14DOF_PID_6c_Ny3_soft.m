%caseB: for the general trajectory, given the curvature, design a minimum-time speed profile along the path, implement the MPC 
% with constraints and X Y Psi as objectives and PID speed control
clc;
clear;
%% parameters definition
m=1400; % Sprung mass (kg)
Jx=900; % Sprung mass roll inertia (kg.m^2)
Jy=2000; % Sprung mass pitch inertia (kg.m^2)
Jz=2420; % Sprung mass yaw inertia (kg.m^2)
a=1.14; % Distance of sprung mass c.g. from front axle (m)
b=1.4; % Distance of sprung mass c.g. from rear axle (m)
Jxz=90; %Product of inertial of moment X Z
Jw=1;     %tire/wheel roll inertia kg.m^2
g=9.8;
h=0.75;  % Sprung mass c.g. height (m)
cf=1.5; 
cr=1.5; % front/rear track width (m)
muf=80;    %front unsprung mass (kg)
mur=80;    %rear unsprung mass (kg)
ktf=200000;  %front tire stiffness (N/m)
ktr=200000;  %rear tire stiffness (N/m)
krof=29000;  %front roll stiffness (Nm/rad)
kror=29000;  %rear roll stiffness (Nm/rad)
brof=3000;   %front roll damping coefficient (Nm.s/rad)
bror=3000;   %rear roll damping coefficient (Nm.s/rad)
Cf=-44000; %front tire cornering stiffness (N/rad)
Cr=-47000;  %rear tire cornering stiffness (N/rad)
Cxf=5000; %front tire longitudinal stiffness 
Cxr=5000; %rear tire longitudinal stiffness 
r0=0.285; %nominal tire radius (m)
hrcf=0.65; %front roll center distance below sprung mass c.g.
hrcr=0.6;  %rear roll center distance below sprung mass c.g.
hrc=(hrcf*b+hrcr*a)/(a+b);
% Td=0.5; %the driving torque.
% Tb=0;   %the braking torque.
mt=m+2*muf+2*mur; %vehicle total mass

%% STATE PREDICTION
Np=30;  % prediction horizon
Nc=25;  % control horizon
Nu=1; % mc denotes the dimension of input
Nx=10; % ns denotes the dimension of state variable
Ny=3; % mo denotes the dimension of outPathput
% Row=1; % Np=10, V0=0.5029 
Row1=1000;Row2=1;Row3=1;Row4=1;%
%% tracking path definition 
%1. genearal trajectory and speed profile along the entire path 
[obj,arc_l,s_l]=references1;
ds=s_l;
Xp=obj(1,:);
Yp=obj(2,:);
Psi=obj(3,:);
Kr=obj(4,:);
Vprs=obj(5,:);
Ax=obj(6,:);
St=arc_l;

%% % initial condition
Vx=Vprs(1);
uc=0;
ct=0;
% the initial tire compression xtif
xtirf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtirr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtlf=xtilf;
xtrf=xtirf;
xtlr=xtilr;
xtrr=xtirr;
wlf=Vx/(r0-xtlf);wrf=Vx/(r0-xtrf);wlr=Vx/(r0-xtlr);wrr=Vx/(r0-xtrr); 
s_lfx=0.2;
s_rfx=0.2;
s_lrx=0.2;
s_rrx=0.2;
u_dot=0;
v_dot=0;
x=[Vx;0;0;0;0;0;0;0;0;0;wlf;wrf;wlr;wrr;0.1];
Tt1=0;Ta=0;Tb=0;
% sf=0;sr=0;wf=Vx/r0;wr=Vx/r0;
xdot(ct+1)=x(1);
ydot(ct+1)=x(2);
phi1(ct+1)=x(3);
psi1(ct+1)=x(4);
wx1(ct+1)=x(5);           
wz1(ct+1)=x(6);
lat_disp(ct+1)=x(7);
long_disp(ct+1)=x(8);
epsi1(ct+1)=x(9);
ey1(ct+1)=x(10);
delta_f(ct+1)=uc;
Tra(ct+1)=Ta;
Trb(ct+1)=Tb;
e_vx(ct+1)=xdot(ct+1)-Vprs(ct+1);
% e_lat_disp(ct+1)=lat_disp(ct+1)-Yp(ct+1);
% e_long_disp(ct+1)=long_disp(ct+1)-Xp(ct+1);
e_psi(ct+1)=psi1(ct+1)-Psi(ct+1);
Ay(ct+1)=0;
dvy=0;
dwz=0;
dwx=0;
beta(ct+1)=atan(x(2)/x(1));
%% calculation for the MPC 
for st=0:ds:St-6-2*ds         
    u=x(1);
    v=x(2);
    phi=x(3);
    psi=x(4);
    wx=x(5);
    wz=x(6);    
    Y=x(7);
    X=x(8);    
    epsi=x(9);
    ey=x(10);
    Ks=Kr(ct+1);
    ux=u;
    wlf=x(11);
    wrf=x(12);
    wlr=x(13);
    wrr=x(14);
    dphi=wx;
    dpsi=wz;
    kesi=zeros(Nx+Nu,1);
    kesi(1)=x(1);%x_dot;
    kesi(2)=x(2);%y_dot;
    kesi(3)=x(3);%phi; is varied along the curv
    kesi(4)=x(4);%psi;
    kesi(5)=x(5);%phi_dot;
    kesi(6)=x(6);%psi_dot;
    kesi(7)=x(7);%Y
    kesi(8)=x(8);%X
    kesi(9)=x(9);
    kesi(10)=x(10);
    kesi(11)=uc;   
    delta=uc;   
    Q_cell=cell(Np,Np);
        for i=1:1:Np
            for j=1:1:Np
                if i==j    
                    Q_cell{i,j}=[22000 0 0;0 500 0;0 0 500];
                else 
                    Q_cell{i,j}=zeros(Ny,Ny);               
                end
            end 
        end 
    Q=cell2mat(Q_cell);
    Rb=50000*eye(Nu*Nc); % Np=30

    A=[                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (((25.6*sin(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (25.6*sin(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)) + (cos(epsi)*(Ks*ey - 1.0)*(2.91*s_lrx + 2.91*s_rrx + v*wz - 1.02*wx*wz - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.91*s_lfx*cos(delta) + 2.91*s_rfx*cos(delta) - 0.0121*wz^2))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                         - (1.0*(Ks*ey - 1.0)*(wz + (25.6*sin(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (25.6*sin(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(Ks*ey - 1.0)*(2.91*s_lrx + 2.91*s_rrx + v*wz - 1.02*wx*wz - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.91*s_lfx*cos(delta) + 2.91*s_rfx*cos(delta) - 0.0121*wz^2))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,                                                                                  0, (1.02*wz*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                        -(1.0*(Ks*ey - 1.0)*(v - 1.02*wx - 0.0242*wz + (25.6*sin(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) + (25.6*sin(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0,                                                                                                                                                                                                                                                                                               -(1.0*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0)*(2.91*s_lrx + 2.91*s_rrx + v*wz - 1.02*wx*wz - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.91*s_lfx*cos(delta) + 2.91*s_rfx*cos(delta) - 0.0121*wz^2))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                               -(1.0*Ks*(2.91*s_lrx + 2.91*s_rrx + v*wz - 1.02*wx*wz - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 5.81e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.91*s_lfx*cos(delta) + 2.91*s_rfx*cos(delta) - 0.0121*wz^2))/(u*cos(epsi) - 1.0*v*sin(epsi));
  - (1.0*(Ks*ey - 1.0)*((40.3*(v - 1.4*wz))/((u - 0.75*wz)^2*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) - 1.0*wz + (40.3*(v - 1.4*wz))/((u + 0.75*wz)^2*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) + (36.5*cos(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (36.5*cos(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (0.388*sin(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (0.388*sin(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*cos(epsi)*(Ks*ey - 1.0)*(25.2*phi - 0.044*s_lrx + 0.044*s_rrx + 3.06*wx + 40.3*atan((v - 1.4*wz)/(u - 0.75*wz)) + 40.3*atan((v - 1.4*wz)/(u + 0.75*wz)) + 1.0*u*wz - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 0.044*s_lfx*cos(delta) + 0.044*s_rfx*cos(delta) - 4.14*s_lfx*sin(delta) - 4.14*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,        ((Ks*ey - 1.0)*(40.3/((u - 0.75*wz)*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) + 40.3/((u + 0.75*wz)*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) + (36.5*cos(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (36.5*cos(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (0.388*sin(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (0.388*sin(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) + (sin(epsi)*(Ks*ey - 1.0)*(25.2*phi - 0.044*s_lrx + 0.044*s_rrx + 3.06*wx + 40.3*atan((v - 1.4*wz)/(u - 0.75*wz)) + 40.3*atan((v - 1.4*wz)/(u + 0.75*wz)) + 1.0*u*wz - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 0.044*s_lfx*cos(delta) + 0.044*s_rfx*cos(delta) - 4.14*s_lfx*sin(delta) - 4.14*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,  (25.2*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                  0,    (3.06*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),         ((Ks*ey - 1.0)*(1.0*u - (40.3*(1.4/(u - 0.75*wz) - (0.75*(v - 1.4*wz))/(u - 0.75*wz)^2))/((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0) - (40.3*(1.4/(u + 0.75*wz) + (0.75*(v - 1.4*wz))/(u + 0.75*wz)^2))/((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0) + (36.5*cos(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) + (36.5*cos(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0) - (0.388*sin(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) + (0.388*sin(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0,          ((v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0)*(25.2*phi - 0.044*s_lrx + 0.044*s_rrx + 3.06*wx + 40.3*atan((v - 1.4*wz)/(u - 0.75*wz)) + 40.3*atan((v - 1.4*wz)/(u + 0.75*wz)) + 1.0*u*wz - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 0.044*s_lfx*cos(delta) + 0.044*s_rfx*cos(delta) - 4.14*s_lfx*sin(delta) - 4.14*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,          (Ks*(25.2*phi - 0.044*s_lrx + 0.044*s_rrx + 3.06*wx + 40.3*atan((v - 1.4*wz)/(u - 0.75*wz)) + 40.3*atan((v - 1.4*wz)/(u + 0.75*wz)) + 1.0*u*wz - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.29e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.81e-6*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 0.044*s_lfx*cos(delta) + 0.044*s_rfx*cos(delta) - 4.14*s_lfx*sin(delta) - 4.14*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (wx*cos(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            -(1.0*wx*sin(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,                                                                                  0,    -(1.0*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -(1.0*wx*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -(1.0*Ks*wx)/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                (wz*cos(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            -(1.0*wz*sin(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,                                                                                  0,                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -(1.0*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -(1.0*wz*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              -(1.0*Ks*wz)/(u*cos(epsi) - 1.0*v*sin(epsi));
 (cos(epsi)*(Ks*ey - 1.0)*(0.123*s_lrx - 49.4*phi - 0.123*s_rrx - 6.0*wx - 26.1*atan((v - 1.4*wz)/(u - 0.75*wz)) - 26.1*atan((v - 1.4*wz)/(u + 0.75*wz)) + 4.73e-20*u*wz + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 0.123*s_lfx*cos(delta) - 0.123*s_rfx*cos(delta) + 2.37*s_lfx*sin(delta) + 2.37*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - (1.0*(Ks*ey - 1.0)*(4.73e-20*wz + (26.1*(v - 1.4*wz))/((u - 0.75*wz)^2*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (26.1*(v - 1.4*wz))/((u + 0.75*wz)^2*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) + (20.8*cos(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (20.8*cos(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (1.08*sin(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (1.08*sin(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)),  ((Ks*ey - 1.0)*(26.1/((u - 0.75*wz)*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) + 26.1/((u + 0.75*wz)*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) + (20.8*cos(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (20.8*cos(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (1.08*sin(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (1.08*sin(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(Ks*ey - 1.0)*(0.123*s_lrx - 49.4*phi - 0.123*s_rrx - 6.0*wx - 26.1*atan((v - 1.4*wz)/(u - 0.75*wz)) - 26.1*atan((v - 1.4*wz)/(u + 0.75*wz)) + 4.73e-20*u*wz + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 0.123*s_lfx*cos(delta) - 0.123*s_rfx*cos(delta) + 2.37*s_lfx*sin(delta) + 2.37*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,  (49.4*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                  0,     (6.0*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)), -(1.0*(Ks*ey - 1.0)*(4.73e-20*u + (26.1*(1.4/(u - 0.75*wz) - (0.75*(v - 1.4*wz))/(u - 0.75*wz)^2))/((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0) + (26.1*(1.4/(u + 0.75*wz) + (0.75*(v - 1.4*wz))/(u + 0.75*wz)^2))/((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0) - (20.8*cos(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) - (20.8*cos(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0) + (1.08*sin(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) - (1.08*sin(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0, -(1.0*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0)*(0.123*s_lrx - 49.4*phi - 0.123*s_rrx - 6.0*wx - 26.1*atan((v - 1.4*wz)/(u - 0.75*wz)) - 26.1*atan((v - 1.4*wz)/(u + 0.75*wz)) + 4.73e-20*u*wz + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 0.123*s_lfx*cos(delta) - 0.123*s_rfx*cos(delta) + 2.37*s_lfx*sin(delta) + 2.37*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2, -(1.0*Ks*(0.123*s_lrx - 49.4*phi - 0.123*s_rrx - 6.0*wx - 26.1*atan((v - 1.4*wz)/(u - 0.75*wz)) - 26.1*atan((v - 1.4*wz)/(u + 0.75*wz)) + 4.73e-20*u*wz + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.73e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 2.46e-5*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 0.123*s_lfx*cos(delta) - 0.123*s_rfx*cos(delta) + 2.37*s_lfx*sin(delta) + 2.37*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
           ((Ks*ey - 1.0)*(4.3e-18*wz + (27.8*(v - 1.4*wz))/((u - 0.75*wz)^2*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (27.8*(v - 1.4*wz))/((u + 0.75*wz)^2*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (20.3*cos(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) - (20.3*cos(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (13.7*sin(delta)*(v + 1.14*wz))/((u - 0.75*wz)^2*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (13.7*sin(delta)*(v + 1.14*wz))/((u + 0.75*wz)^2*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) + (cos(epsi)*(Ks*ey - 1.0)*(1.62*phi - 1.55*s_lrx + 1.55*s_rrx + 0.197*wx + 27.8*atan((v - 1.4*wz)/(u - 0.75*wz)) + 27.8*atan((v - 1.4*wz)/(u + 0.75*wz)) - 4.3e-18*u*wz + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 1.55*s_lfx*cos(delta) + 1.55*s_rfx*cos(delta) + 2.3*s_lfx*sin(delta) + 2.3*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2, - (1.0*(Ks*ey - 1.0)*(27.8/((u - 0.75*wz)*((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0)) + 27.8/((u + 0.75*wz)*((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (20.3*cos(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) - (20.3*cos(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)) - (13.7*sin(delta))/((u - 0.75*wz)*((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0)) + (13.7*sin(delta))/((u + 0.75*wz)*((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0))))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(Ks*ey - 1.0)*(1.62*phi - 1.55*s_lrx + 1.55*s_rrx + 0.197*wx + 27.8*atan((v - 1.4*wz)/(u - 0.75*wz)) + 27.8*atan((v - 1.4*wz)/(u + 0.75*wz)) - 4.3e-18*u*wz + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 1.55*s_lfx*cos(delta) + 1.55*s_rfx*cos(delta) + 2.3*s_lfx*sin(delta) + 2.3*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2, -(1.62*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                  0,  -(0.197*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),       ((Ks*ey - 1.0)*(4.3e-18*u + (27.8*(1.4/(u - 0.75*wz) - (0.75*(v - 1.4*wz))/(u - 0.75*wz)^2))/((v - 1.4*wz)^2/(u - 0.75*wz)^2 + 1.0) + (27.8*(1.4/(u + 0.75*wz) + (0.75*(v - 1.4*wz))/(u + 0.75*wz)^2))/((v - 1.4*wz)^2/(u + 0.75*wz)^2 + 1.0) + (20.3*cos(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) + (20.3*cos(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0) + (13.7*sin(delta)*(1.14/(u - 0.75*wz) + (0.75*(v + 1.14*wz))/(u - 0.75*wz)^2))/((v + 1.14*wz)^2/(u - 0.75*wz)^2 + 1.0) - (13.7*sin(delta)*(1.14/(u + 0.75*wz) - (0.75*(v + 1.14*wz))/(u + 0.75*wz)^2))/((v + 1.14*wz)^2/(u + 0.75*wz)^2 + 1.0)))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0,      -(1.0*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0)*(1.62*phi - 1.55*s_lrx + 1.55*s_rrx + 0.197*wx + 27.8*atan((v - 1.4*wz)/(u - 0.75*wz)) + 27.8*atan((v - 1.4*wz)/(u + 0.75*wz)) - 4.3e-18*u*wz + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 1.55*s_lfx*cos(delta) + 1.55*s_rfx*cos(delta) + 2.3*s_lfx*sin(delta) + 2.3*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,      -(1.0*Ks*(1.62*phi - 1.55*s_lrx + 1.55*s_rrx + 0.197*wx + 27.8*atan((v - 1.4*wz)/(u - 0.75*wz)) + 27.8*atan((v - 1.4*wz)/(u + 0.75*wz)) - 4.3e-18*u*wz + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 4.61e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 3.11e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 1.55*s_lfx*cos(delta) + 1.55*s_rfx*cos(delta) + 2.3*s_lfx*sin(delta) + 2.3*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          (cos(epsi)*(Ks*ey - 1.0)*(v*cos(psi) + u*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - (1.0*sin(psi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     - (1.0*cos(psi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(Ks*ey - 1.0)*(v*cos(psi) + u*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0, -(1.0*(u*cos(psi) - 1.0*v*sin(psi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       -(1.0*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0)*(v*cos(psi) + u*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       -(1.0*Ks*(v*cos(psi) + u*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      (cos(epsi)*(u*cos(psi) - 1.0*v*sin(psi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - (1.0*cos(psi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (sin(psi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(u*cos(psi) - 1.0*v*sin(psi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,          ((Ks*ey - 1.0)*(v*cos(psi) + u*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   -(1.0*(u*cos(psi) - 1.0*v*sin(psi))*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   -(1.0*Ks*(u*cos(psi) - 1.0*v*sin(psi)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    (cos(epsi)*(wz + (Ks*(u*cos(epsi) - 1.0*v*sin(epsi)))/(Ks*ey - 1.0))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - (1.0*Ks*cos(epsi))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     (Ks*sin(epsi))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(wz + (Ks*(u*cos(epsi) - 1.0*v*sin(epsi)))/(Ks*ey - 1.0))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,                                                                                  0,                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  -(1.0*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)), 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                     (Ks*(v*cos(epsi) + u*sin(epsi)))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*(wz + (Ks*(u*cos(epsi) - 1.0*v*sin(epsi)))/(Ks*ey - 1.0))*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   Ks^2/(Ks*ey - 1.0) - (1.0*Ks*(wz + (Ks*(u*cos(epsi) - 1.0*v*sin(epsi)))/(Ks*ey - 1.0)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       (cos(epsi)*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - (1.0*sin(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)),                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  - (1.0*cos(epsi)*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi)) - (1.0*sin(epsi)*(v*cos(epsi) + u*sin(epsi))*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2,                                                     0,                                                                                  0,                                                       0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     0, 0, 0,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              1.0 - (1.0*(v*cos(epsi) + u*sin(epsi))^2*(Ks*ey - 1.0))/(u*cos(epsi) - 1.0*v*sin(epsi))^2 - 1.0*Ks*ey,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     -(1.0*Ks*(v*cos(epsi) + u*sin(epsi)))/(u*cos(epsi) - 1.0*v*sin(epsi))];
    
    
    B=[ 
                                                                                                                                                                                                                ((Ks*ey - 1.0)*(51.2*sin(delta) + 5.81e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 5.81e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.91*s_lfx*sin(delta) + 2.91*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
 -(1.0*(Ks*ey - 1.0)*(72.9*cos(delta) - 8.81e-6*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 8.81e-6*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 8.29e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 8.29e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 4.14*s_lfx*cos(delta) + 4.14*s_rfx*cos(delta) - 0.044*s_lfx*sin(delta) + 0.044*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0;
 -(1.0*(Ks*ey - 1.0)*(41.6*cos(delta) - 2.46e-5*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) + 2.46e-5*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 4.73e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 4.73e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.37*s_lfx*cos(delta) + 2.37*s_rfx*cos(delta) - 0.123*s_lfx*sin(delta) + 0.123*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
     -(1.0*(Ks*ey - 1.0)*(40.5*cos(delta) + 3.11e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 3.11e-4*cos(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) - 4.61e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u - 0.75*wz))) - 4.61e-4*sin(delta)*(4.4e4*delta - 4.4e4*atan((v + 1.14*wz)/(u + 0.75*wz))) + 2.3*s_lfx*cos(delta) + 2.3*s_rfx*cos(delta) + 1.55*s_lfx*sin(delta) - 1.55*s_rfx*sin(delta)))/(u*cos(epsi) - 1.0*v*sin(epsi));
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                        0];
    
    
    
    
    % arc length 'S' discretization of the internal predictive model
    Ip=eye(Nx);
    Ad=Ip+A*ds;
    Bd=B*ds;      
    %%
% the instantaneous tire radius (linearization of trigonometric terms)
    Rrf=r0-xtrf;
    Rlf=r0-xtlf;
    Rlr=r0-xtlr;
    Rrr=r0-xtrr;
% position of front and rear unsprung mass
    huf=Rrf; 
    hur=Rrr;   
% %the longitudinal and lateral velocities at the tire contact patch in coordinate frame 2
    ugrf=u+(wz*cf)/2;
    vgrf=v+wz*a;
    uglf=u-(wz*cf)/2;
    vglf=v+wz*a;
    uglr=u-(wz*cr)/2;
    vglr=v-wz*b;
    ugrr=u+(wz*cr)/2;
    vgrr=v-wz*b;
% tire slip angle of each wheel
    delta_rf=atan(vgrf/ugrf)-delta;
    delta_lf=atan(vglf/uglf)-delta;
    delta_lr=atan(vglr/uglr);
    delta_rr=atan(vgrr/ugrr);
%linear tire lateral force
    Fytrf=Cf*delta_rf;
    Fytlf=Cf*delta_lf;
    Fytlr=Cr*delta_lr;
    Fytrr=Cr*delta_rr;
% longitudinal slips
    if Rrf*wrf>(ugrf*cos(delta)+vgrf*sin(delta))
    s_rf=1-abs((ugrf*cos(delta)+vgrf*sin(delta))/(Rrf*wrf));
    else 
    s_rf=abs((Rrf*wrf)/(ugrf*cos(delta)+vgrf*sin(delta)))-1;
    end
    if Rlf*wlf>(uglf*cos(delta)+vglf*sin(delta))
    s_lf=1-abs((uglf*cos(delta)+vglf*sin(delta))/(Rlf*wlf));
    else 
    s_lf=abs((Rlf*wlf)/(uglf*cos(delta)+vglf*sin(delta)))-1;
    end
    if Rlr*wlr>uglr
    s_lr=1-abs(uglr/(Rlr*wlr));
    else 
    s_lr=abs((Rlr*wlr)/uglr)-1;
    end
    if Rrr*wrr>ugrr
    s_rr=1-abs(ugrr/(Rrr*wrr));
    else
    s_rr=abs((Rrr*wrr)/ugrr)-1;
    end
% linear tire longitudinal force 
    Fxtrf=Cxf*s_rf;
    Fxtlf=Cxf*s_lf;
    Fxtlr=Cxr*s_lr;
    Fxtrr=Cxr*s_rr;
% the forces Fxgij are obtained by resolving the longitudinal and cornering forces at the tire contact patch 
    Fxglf=Fxtlf*cos(delta)-Fytlf*sin(delta);
    Fxgrf=Fxtrf*cos(delta)-Fytrf*sin(delta);
    Fxglr=Fxtlr;
    Fxgrr=Fxtrr;
    Fyglf=Fxtlf*sin(delta)+Fytlf*cos(delta);
    Fygrf=Fxtrf*sin(delta)+Fytrf*cos(delta);
    Fyglr=Fytlr;
    Fygrr=Fytrr;
% The normal forces at four tires are determined as
    mtuf=2*muf; mtur=2*mur;
    Z1=(m*g*b)/(2*(a+b))+(mtuf*g)/2;
    Z2=(v_dot+u*wz)*((huf*mtuf)/cf+(b*m*(h-hrcf))/(cf*(a+b)));
    Z3=(krof*phi+brof*dphi)/cf;
    Z4=((m*h+mtuf*huf+mtur*hur)*(u_dot-wz*v))/(2*(a+b));
    Fzglf=Z1-Z2-Z3-Z4;
    Fzgrf=Z1+Z2+Z3-Z4;
    Z5=(m*g*a)/(2*(a+b))+(mtur*g)/2;
    Z6=(v_dot+u*wz)*((hur*mtur)/cr+(a*m*(h-hrcr))/(cr*(a+b)));
    Z7=(kror*phi+bror*dphi)/cr;
    Z8=((m*h+mtuf*huf+mtur*hur)*(u_dot-wz*v))/(2*(a+b));
    Fzglr=Z5-Z6-Z7+Z8;
    Fzgrr=Z5+Z6+Z7+Z8;  
    LTR_x=(Fzgrf+Fzgrr-Fzglf-Fzglr)/((m+mtuf+mtur)*g);% calculate the load transfer ratio

    %% the wheel rotational modeling
    s_dot=(u*cos(epsi)-v*sin(epsi))/(1-Ks*ey);    
%     dwlf=(1/Jw)*(-Fxtlf*Rlf)/s_dot;
%     dwrf=(1/Jw)*(-Fxtrf*Rrf)/s_dot;
%     dwlr=(1/Jw)*(-Fxtlr*Rlr)/s_dot;
%     dwrr=(1/Jw)*(-Fxtrr*Rrr)/s_dot;   
%     
%     wlf=wlf+ds*dwlf;
%     wrf=wrf+ds*dwrf;
%     wlr=wlr+ds*dwlr;
%     wrr=wrr+ds*dwrr;

 %% solution for roll angle subject to LTR=1
    B=(cf+cr)/2;
%   Ay=(g*((B/2)-roll_u*hrc))/h;
    LT1=2*((hur*mtur)/cr+(a*m*(h-hrcr))/(cr*(a+b)));
    LT2=2*((huf*mtuf)/cf+(b*m*(h-hrcf))/(cf*(a+b)));
    LT3=2*(brof/cf+bror/cr);
    LT4=2*(krof/cf+kror/cr);
    LT5=(m+mtuf+mtur)*g;
    tspan=[0 (ds/s_dot)*Np];
    roll_i=0;
    [t,roll_u]=ode45(@(t,roll_u) (1/LT3)*(LT5-LT4*roll_u-(LT1*g*(B/2-hrc*roll_u))/h-(LT2*g*(B/2-hrc*roll_u))/h), tspan, roll_i);
    roll_limit=roll_u(length(roll_u));  
    
%% vehicle body dynamics module    
    dphi=wx;
    dpsi=wz;
    E1=-mt*wz*u+(Fyglf+Fygrf+Fyglr+Fygrr);
    E2=(Fyglf+Fygrf)*a-(Fyglr+Fygrr)*b+(Fxgrf-Fxglf)*cf/2+(Fxgrr-Fxglr)*cr/2+(mur*b-muf*a)*wz*u;
    E3=m*g*hrc*phi-(krof+kror)*phi-(brof+bror)*dphi+hrc*m*wz*u;
    A1=mur*b-muf*a;
    A2=Jx+m*hrc^2;
    A3=hrc*m;
    u_dot = wz*v+(1/mt)*((Fxglf+Fxgrf+Fxglr+Fxgrr)+(muf*a-mur*b)*(wz)^2-2*hrc*m*wz*wx);
    v_dot = (E1*Jxz^2-A1*A2*E2+A1*E3*Jxz+A3*E2*Jxz-A2*E1*Jz-A3*E3*Jz)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
    wx_dot= (A1^2*E3-A1*A3*E2+A1*E1*Jxz-A3*E1*Jz+E2*Jxz*mt-E3*Jz*mt)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
    wz_dot= (A3^2*E2-A1*A2*E1-A1*A3*E3+A3*E1*Jxz-A2*E2*mt+E3*Jxz*mt)/(A2*A1^2-2*A1*A3*Jxz+Jz*A3^2+mt*Jxz^2-A2*Jz*mt);
    Y_dot=u*sin(psi)+v*cos(psi);
    X_dot=u*cos(psi)-v*sin(psi);  
%     s_dot=(u*cos(epsi)-v*sin(epsi))/(1-Ks*ey); 
%     epsi_dot=wz-Ks*(u*cos(epsi)-v*sin(epsi))/(1-Ks*ey);      
    epsi_dot=wz-s_dot*Ks;
    ey_dot=u*sin(epsi)+v*cos(epsi);
    
    u_sdot=u_dot/s_dot;
    v_sdot=v_dot/s_dot;
    dphi_s=wx/s_dot;
    dpsi_s=wz/s_dot;
    wx_sdot=wx_dot/s_dot;
    wz_sdot=wz_dot/s_dot;
    Y_sdot=Y_dot/s_dot;
    X_sdot=X_dot/s_dot;
    epsi_sdot=epsi_dot/s_dot;
    ey_sdot=ey_dot/s_dot;
    
%%    
    state_k1=zeros(Nx,1);
    state_k1(1,1)=u+ds*u_sdot;
    state_k1(2,1)=v+ds*v_sdot;
    state_k1(3,1)=phi+ds*dphi_s;
    state_k1(4,1)=psi+ds*dpsi_s;
    state_k1(5,1)=wx+ds*wx_sdot;
    state_k1(6,1)=wz+ds*wz_sdot;
    state_k1(7,1)=Y+ds*Y_sdot;
    state_k1(8,1)=X+ds*X_sdot;            
    state_k1(9,1)=epsi+ds*epsi_sdot;
    state_k1(10,1)=ey+ds*ey_sdot;
    
    d_k=state_k1-Ad*kesi(1:10,1)-Bd*kesi(11,1);%
    d_piao_k=zeros(Nx+Nu,1);
    d_piao_k(1:10,1)=d_k;
    d_piao_k(11,1)=0;
    
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=Ad;
    A_cell{1,2}=Bd;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=Bd;
    B_cell{2,1}=eye(Nu);
    An=cell2mat(A_cell);
    Bn=cell2mat(B_cell);
    
%% outputs for cost function
    C=[0 0 0 1 0 0 0 0 0 0;
       0 0 0 0 0 0 1 0 0 0;
       0 0 0 0 0 0 0 1 0 0];
    Zm=zeros(Ny,Nu);
    Cn=[C Zm];
    PSI_cell=cell(Np,1);
    THETA_cell=cell(Np,Nc);
    GAMMA_cell=cell(Np,Np);
    PHI_cell=cell(Np,1);
    for p=1:1:Np
        PHI_cell{p,1}=d_piao_k;%
        for q=1:1:Np
            if q<=p
                GAMMA_cell{p,q}=Cn*An^(p-q);
            else 
                GAMMA_cell{p,q}=zeros(Ny,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_cell{j,1}=Cn*An^j;
        for k=1:1:Nc
            if k<=j
                THETA_cell{j,k}=Cn*An^(j-k)*Bn;
            else 
                THETA_cell{j,k}=zeros(Ny,Nu);
            end
        end
    end
    PSI=cell2mat(PSI_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA=cell2mat(THETA_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA=cell2mat(GAMMA_cell);%GAMMA
    PHI=cell2mat(PHI_cell);
  
    H_cell=cell(2,2);
    H_cell{1,1}=THETA'*Q*THETA+Rb;
    H_cell{1,2}=zeros(Nu*Nc,4);
    H_cell{2,1}=zeros(4,Nu*Nc);
    H_cell{2,2}=[Row1 0 0 0;0 Row2 0 0;0 0 Row3 0;0 0 0 Row4];
    H=cell2mat(H_cell);
    error_1=zeros(Ny*Np,1);
    Yita_ref_cell=cell(Np,1);      
    
    for p=1:1:Np
    Y_ref(p,1)=Yp(p+ct); 
    X_ref(p,1)=Xp(p+ct);
    psi_ref(p,1)=Psi(p+ct);
  Yita_ref_cell{p,1}=[psi_ref(p,1);Y_ref(p,1);X_ref(p,1)];
    end  
    Yita_ref=cell2mat(Yita_ref_cell);    
    error_1=Yita_ref-PSI*kesi-GAMMA*PHI; 
    f_cell=cell(1,2);
    f_cell{1,1}=2*error_1'*Q*THETA;
    f_cell{1,2}=[0 0 0 0];
    f=-cell2mat(f_cell);  

%% state output for lateral and yaw stability
    Ny_sta=3;
    C_sta=[0 1/ux 0 0 0 -b/ux 0 0 0 0;
           0 1/ux 0 0 0 0 0 0 0 0;
           0 0 0 0 0 1 0 0 0 0];
    Zm_sta=zeros(Ny_sta,Nu);
    Cn_sta=[C_sta Zm_sta];
    PSI_sta_cell=cell(Np,1);
    THETA_sta_cell=cell(Np,Nc);
    GAMMA_sta_cell=cell(Np,Np);
    PHI_sta_cell=cell(Np,1);
    for p=1:1:Np
        PHI_sta_cell{p,1}=d_piao_k;%
        for q=1:1:Np
            if q<=p
                GAMMA_sta_cell{p,q}=Cn_sta*An^(p-q);
            else 
                GAMMA_sta_cell{p,q}=zeros(Ny_sta,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_sta_cell{j,1}=Cn_sta*An^j;
        for k=1:1:Nc
            if k<=j
                THETA_sta_cell{j,k}=Cn_sta*An^(j-k)*Bn;
            else 
                THETA_sta_cell{j,k}=zeros(Ny_sta,Nu);
            end
        end
    end
    PSI_sta=cell2mat(PSI_sta_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA_sta=cell2mat(THETA_sta_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA_sta=cell2mat(GAMMA_sta_cell);%GAMMA
    PHI_sta=cell2mat(PHI_sta_cell);            
 
  %% state output for rollover prevention
    Ny_rol=1;
    C_rol=[0 0 1 0 0 0 0 0 0 0];
    Zm_rol=zeros(Ny_rol,Nu);
    Cn_rol=[C_rol Zm_rol];
    PSI_rol_cell=cell(Np,1);
    THETA_rol_cell=cell(Np,Nc);
    GAMMA_rol_cell=cell(Np,Np);
    PHI_rol_cell=cell(Np,1);
    for p=1:1:Np
        PHI_rol_cell{p,1}=d_piao_k;%
        for q=1:1:Np
            if q<=p
                GAMMA_rol_cell{p,q}=Cn_rol*An^(p-q);
            else 
                GAMMA_rol_cell{p,q}=zeros(Ny_rol,Nx+Nu);
            end 
        end
    end
    for j=1:1:Np
     PSI_rol_cell{j,1}=Cn_rol*An^j;
        for k=1:1:Nc
            if k<=j
                THETA_rol_cell{j,k}=Cn_rol*An^(j-k)*Bn;
            else 
                THETA_rol_cell{j,k}=zeros(Ny_rol,Nu);
            end
        end
    end
    PSI_rol=cell2mat(PSI_rol_cell);%size(PSI)=[Ny*Np Nx+Nu]
    THETA_rol=cell2mat(THETA_rol_cell);%size(THETA)=[Ny*Np Nu*Nc]
    GAMMA_rol=cell2mat(GAMMA_rol_cell);%GAMMA
    PHI_rol=cell2mat(PHI_rol_cell);  
    
%% constraints definition for cost function   
    A_t=zeros(Nc,Nc);%
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    A_I=kron(A_t,eye(Nu));%
    Ut=kron(ones(Nc,1),uc);%2000
    umin=-0.0436*5;%
    umax=0.0436*5;
    delta_umin=-0.0148*5;
    delta_umax=0.0148*5;
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);

% general trajectory output constraints    
    ycmax=[1;10;60];
    ycmin=[-6;-90;-40];
% 8-shaped curve output constraints   
%     ycmax=[3;10;200];
%     ycmin=[-3;-500;-10];    
    Ycmax=kron(ones(Np,1),ycmax);
    Ycmin=kron(ones(Np,1),ycmin);

%% for lateral and yaw stability
    mu=0.9;L=a+b;
    afa_sat=abs(atan(3*mt*g*mu*a/(Cr*L)));
    beta_max=atan(0.02*mu*g);
    dpsi_m1=abs(Cr*abs(afa_sat)*(1+b/a)/(mt*ux));
    dpsi_m2=0.85*mu*g/ux;
    yawrate=min([dpsi_m1,dpsi_m2]);
    ymax_sta=[afa_sat;beta_max;yawrate];
    ymin_sta=[-afa_sat;-beta_max;-yawrate];
    Ymax_sta=kron(ones(Np,1),ymax_sta);
    Ymin_sta=kron(ones(Np,1),ymin_sta);

%% for rollover prevention
    roll_max=abs(roll_limit);
    roll_min=-abs(roll_limit);
    Ymax_rol=kron(ones(Np,1),roll_max);
    Ymin_rol=kron(ones(Np,1),roll_min);

%%
    A_cons_cell={A_I zeros(Nu*Nc,4);-A_I zeros(Nu*Nc,4);THETA zeros(Ny*Np,4);-THETA zeros(Ny*Np,4);THETA_sta [zeros(Ny_sta*Np,1) -ones(Ny_sta*Np,3)];-THETA_sta [zeros(Ny_sta*Np,1) -ones(Ny_sta*Np,3)];THETA_rol zeros(Ny_rol*Np,4);-THETA_rol zeros(Ny_rol*Np,4)};
    b_cons_cell={Umax-Ut;-Umin+Ut;Ycmax-PSI*kesi-GAMMA*PHI;-Ycmin+PSI*kesi+GAMMA*PHI;Ymax_sta-PSI_sta*kesi-GAMMA_sta*PHI_sta;-Ymin_sta+PSI_sta*kesi+GAMMA_sta*PHI_sta;Ymax_rol-PSI_rol*kesi-GAMMA_rol*PHI_rol;-Ymin_rol+PSI_rol*kesi+GAMMA_rol*PHI_rol};
    A_cons=cell2mat(A_cons_cell);
    b_cons=cell2mat(b_cons_cell);

    M1=50; M2=0.0615;M3=0.0175;M4=0.13;  
    delta_Umin=kron(ones(Nc,1),delta_umin);
    delta_Umax=kron(ones(Nc,1),delta_umax);
    lb=[delta_Umin;zeros(4,1)];
    ub=[delta_Umax;M1;M2;M3;M4]; 
% %     
    x_start=zeros(Nc+4,1);%
    [du,fval,exitflag]=quadprog(H,f,A_cons,b_cons,[],[],lb,ub,x_start);
% %     
%     du=quadprog(H,f);
    u_piao=du(1);
    uc=kesi(11,1)+u_piao;

%%   for the longitudinal contorller (PID controller)
    Vx=x(1);
    Vy=x(2);
    psi_dot=x(6);
    Axp=x(15);
%     Kp=8500;KI=180;KD=10; %Np=30, 8-shaped curve
      Kp=8500;KI=1200;KD=10; %Np=30, general trajectory
    Vx_e=Vx-Vprs(ct+1);
    x_e=(Vx-Vprs(ct+1))*(ds/s_dot);
    dVx_e=Axp-Ax(ct+1);
    if Vx_e<0
        Ta=-Kp*Vx_e-KI*x_e-KD*dVx_e;    
        Tb=0;
        Tt1=Ta-Tb;
    elseif Vx_e==00
        Ta=0;
        Tb=0;
        Tt1=Ta-Tb;
    else
        Ta=0;
        Tb=Kp*Vx_e+KI*x_e+KD*dVx_e;
        Tt1=Ta-Tb;
    end         
    ct=ct+1;
    Tt=ds/s_dot;
    Xc=[x(1);x(2);x(3);x(4);x(5);x(6);x(7);x(8);x(9);x(10);Tt;Psi(ct+1);dvy;Axp;dwz;dwx;xtlf;xtrf;xtlr;xtrr];
% calculate the state of the plant 
    x=plant_14DOF_vehicle_4([uc;Ta;Tb],Vx,Xc);    
    dvy=x(17);
    xtlf=x(18);
    xtrf=x(19);
    xtlr=x(20);
    xtrr=x(21); 
    LTR=x(22);
    dwz=x(23);
    dwx=x(24);
    phi_0=0;
    [t,phi_0]=ode45(@(t,phi_0) (1/LT3)*(LTR*LT5-(LT1+LT2)*(dvy+x(1)*x(6))-LT4*phi_0),tspan,phi_0);
    phi_0=phi_0(length(phi_0));  
% for the plot
    xdot(ct+1)=x(1);
    ydot(ct+1)=x(2);
    phi1(ct+1)=x(3)+phi_0;
    psi1(ct+1)=x(4);
    wx1(ct+1)=x(5);           
    wz1(ct+1)=x(6);
    lat_disp(ct+1)=x(7);
    long_disp(ct+1)=x(8);
    delta_f(ct+1)=uc;
    Tra(ct+1)=Ta;
    Trb(ct+1)=Tb;
    Trs(ct+1)=Ta-Tb;
    Axp1(ct+1)=Axp;
    Ti(ct+1)=Tt;
    epsi1(ct+1)=x(9);
    ey1(ct+1)=x(10);
    e_vx(ct+1)=xdot(ct+1)-Vprs(ct+1);    
    Ay(ct+1)=x(16);
    roll_up(ct)=roll_max;
    roll_low(ct)=roll_min;
    yawrate_up(ct)=yawrate;
    yawrate_low(ct)=-yawrate;
    LTR_ratio(ct)=LTR;
    LTR_ratio_x(ct)=LTR_x;
    beta(ct+1)=atan(x(2)/x(1));
end
s=0:ds:St-6-ds;
ii=0;
for i=0:ds:St-6-ds
    ii=ii+1;
    Psi_ref(ii)=Psi(ii);
    Yp_ref(ii)=Yp(ii);
    Xp_ref(ii)=Xp(ii);
    Vxp_ref(ii)=Vprs(ii);
end
figure(1)
plot(s,psi1);
grid;
hold on;
plot(s,Psi_ref,'r--');
% title('Heading angle comparison');
xlabel('Distance along path s (m)');
ylabel('Heading angle (rad)');
legend('The plant output','Reference','Location','best');
figure(2)
plot(s,lat_disp);
grid;
hold on;
plot(s,Yp_ref,'r--');
% title('Lateral position of vehicle C.M. comparison');
xlabel('Distance along path s (m)');
ylabel('Lateral position of Vehicle C.M.(m)');
legend('The plant output','Reference','Location','best');
figure(3)
plot(s,long_disp);
grid;
hold on;
plot(s,Xp_ref,'r--');
% title('Longitudinal position of vehicle C.M. comparison');
xlabel('Distance along path s (m)');
ylabel('Longitudinal position of Vehicle C.M.(m)');
legend('The plant output','Reference','Location','northeast');
figure(4)
plot(long_disp,lat_disp);
hold on;
plot(Xp_ref,Yp_ref,'r--');
grid;
axis equal
% axis([0,800,-100,100]);
% title('Trajectory Comparison');
xlabel('X (m)');
ylabel('Y (m)');
legend('The plant output','Reference path','Location','best');
figure(5)
plot(s,xdot); 
grid;
hold on;
plot(s,Vxp_ref,'r--');
axis([0,300,0,45]);
% title('Speed Comparison');
xlabel('Distance along path s (m)');
ylabel('Longitudinal velocity (m/s)');
legend('The plant output','Reference','Location','best');
figure(6)
plot(s,delta_f);
% axis([0,St,-0.2,0.2]);
grid;
% title('front wheel steering angle');
xlabel('Distance along path s (m)');
ylabel('Steering angle (rad)');
figure(7)
plot(s,Tra,'b',s,Trb,'g');
axis([0,300,0,4000]);
grid;
% title('Wheel driving& braking torque');
xlabel('Distance along path s (m)');
ylabel('Driving & braking torque (Nm)');
legend('total driving torque','total braking torque','Location','best');

Vx=obj(9,:);
Vp1=obj(7,:);
Vp2=obj(8,:);
ii=0;
for i=0:ds:St-6-ds
    ii=ii+1;
    Kr_ref(ii)=Kr(ii);
    Vx_ref(ii)=Vx(ii);
    Vp1_ref(ii)=Vp1(ii);
    Vp2_ref(ii)=Vp2(ii);
    Vprs_ref(ii)=Vprs(ii);
end
% plot speed profile
figure;
plot(s,Kr_ref,'r');
grid;
% title('Reference Curvature Profile');
xlabel('Distance along path s (m)');
ylabel('Curvature K (1/m)');
figure;
plot(s,Vx_ref,'g-.',s,Vp1_ref,'b--',s,Vp2_ref,'r');
grid;
axis([0,300,0,45]);
% title('Velocity profile generation');
xlabel('Distance along path s (m)');
ylabel('Longitudinal velocity (m/s)');
legend('Fx=0','Integrate Forward','Integrate Backward');

figure;
plot(s,lat_disp-Yp_ref)
grid;
% title('Lateral position tracking error');
xlabel('Distance along path s (m)');
ylabel('Y Error (m)');

figure;
plot(s,long_disp-Xp_ref)
grid;
% title('Longitudinal position tracking error');
xlabel('Distance along path s (m)');
ylabel('X Error (m)');

figure;
plot(s, xdot-Vxp_ref)
grid;
title('Speed tracking error');
xlabel('Distance along path s (m)');
ylabel('Longitudinal velocity error (m/s)');

figure;
plot(s,Ay);
grid;
title('Vehicle lateral acceleration');
xlabel('Distance along path s (m)');
ylabel('Lateral acceleration (m/s^2)');

figure;
plot(s,phi1);
grid;
title('Vehicle roll angle');
xlabel('Distance along path s (m)');
ylabel('Roll angle(rad)');

figure;
plot(roll_up);
hold on;
plot(roll_low);
plot(phi1);
grid;
