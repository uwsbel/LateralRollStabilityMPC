% FOR 8DOF CONTROLLER
function x=plant_14DOF_vehicle_4(Sss,Vx,Xc)
%% 14 DOF vehicle dynamics model (6 DOF at vehicle lumped mass c.g. and 2 DOF at each of four wheels(vertical suspension travel and wheel spin)
%% state variables definition
% u; % the longitudinal velocity 
% v; % the lateral velocity
% w; % the vertical velocity
% phi; %roll angle
% wx;  %roll angular velocity
% theta; %pitch angle
% wy;    %pitch angular velocity
% psi; %yaw angle
% wz;  %yaw angular velocity
% wlf; % angular velocity of left front wheel rotation 
% wrf; % angular velocity of right front wheel rotation
% wlr; % angular velocity of left rear wheel rotation
% wrr; % angular velocity of right rear wheel rotation
% xtlf; % instantaneous left front tire deflection
% xtrf; % instantaneous right front tire deflection
% xtlr; % instantaneous left rear tire deflection
% xtrr; % instantaneous right rear tire deflection
% xsrf; % instantaneous front right suspension spring deflection
% xslf; % instantaneous front left suspension spring deflection
% xslr; % instantaneous rear left suspension spring deflection
% xsrr; % instantaneous rear right suspension spring deflection
%% parameters definition
m=1400; % Sprung mass (kg)
Jx=900; % Sprung mass roll inertia (kg.m^2)
Jy=2000; % Sprung mass pitch inertia (kg.m^2)
Jz=2420; % Sprung mass yaw inertia (kg.m^2)
Jw=1;     %tire/wheel roll inertia kg.m^2
g=9.8;
a=1.14; % Distance of sprung mass c.g. from front axle (m)
b=1.4; % Distance of sprung mass c.g. from rear axle (m)
h=0.75;  % Sprung mass c.g. height (m)
cf=1.5;
cr=1.5; % front/rear track width (m)
kslf=35000; %front left suspension stiffness (N/m)
ksrf=35000;
kslr=30000; %rear left suspension stiffness (N/m)
ksrr=30000;
bslf=2500;  %front left suspension damping coefficient (Ns/m)
bsrf=2500;
bslr=2000;  %rear left suspension damping coefficient (Ns/m)
bsrr=2000;  
muf=80;    %front unsprung mass (kg)
mur=80;    %rear unsprung mass (kg)
ktf=200000;  %front tire stiffness (N/m)
ktr=200000;  %rear tire stiffness (N/m)
Cf=-44000; %front tire cornering stiffness (N/rad)
Cr=-47000;  %rear tire cornering stiffness (N/rad)
Cxf=5000; %front tire longitudinal stiffness 
Cxr=5000; %rear tire longitudinal stiffness 
r0=0.285; %nominal tire radius (m)
hrcf=0.65; %front roll center distance below sprung mass c.g.
hrcr=0.6;  %rear roll center distance below sprung mass c.g.
% Td=0.5; %the driving torque.
% Tb=0;   % the braking torque.
% T=0.001;% time step
%% steering angle input
Tsim=round(Xc(11)/0.001);
steer_angle=Sss(1)*ones(Tsim,1);
Ta=Sss(2);
Tb=Sss(3);
%% calculate the initial conditions
theta=0;phi=0;psi=Xc(4);
u=Xc(1);v=Xc(2);w=0;
u_dot=Xc(14);v_dot=Xc(13);w_dot=0;
wx=Xc(5);wy=0;wz=Xc(6);
Y=Xc(7);X=Xc(8);
wx_dot=Xc(16);wy_dot=0;wz_dot=Xc(15);
epsi=Xc(9);ey=Xc(10);
% wlf=u/r0;wrf=u/r0;wlr=u/r0;wrr=u/r0;
% the initial tire compression xtif
xtirf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilf=((m*g*b)/(2*(a+b))+muf*g)/ktf;
xtilr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtirr=((m*g*a)/(2*(a+b))+mur*g)/ktr;
xtlf=Xc(17);
xtrf=Xc(18);
xtlr=Xc(19);
xtrr=Xc(20);

%the initial spring compression xsif 
xsirf=(m*g*b)/(2*(a+b)*ksrf);
xsilf=(m*g*b)/(2*(a+b)*kslf);
xsilr=(m*g*a)/(2*(a+b)*kslr);
xsirr=(m*g*a)/(2*(a+b)*ksrr);
xsrf=xsirf;
xslf=xsilf;
xslr=xsilr;
xsrr=xsirr;
% the initial length of the strut
lsirf=h-(r0-xtirf);
lsilf=h-(r0-xtilf);
lsilr=h-(r0-xtilr);
lsirr=h-(r0-xtirr);
lsrf=lsirf;
lslf=lsilf;
lslr=lsilr;
lsrr=lsirr;
% the transforming matrix 
Mrf=[0 0 cf/2;0 0 a;-cf/2 -a 0];
Mlf=[0 0 -cf/2;0 0 a;cf/2 -a 0];
Mlr=[0 0 -cr/2;0 0 -b;cr/2 b 0];
Mrr=[0 0 cr/2;0 0 -b;-cr/2 b 0];
% initial velocity of strut mounting point in x y z by transforming the c.g. velocities
wm=[wx;wy;wz];
vm=[u;v;w];
Vsrf=Mrf*wm+vm; % front right mounting point x y z velocity in coordinate frame 1
Vslf=Mlf*wm+vm; % front left
Vslr=Mlr*wm+vm; % rear left
Vsrr=Mrr*wm+vm; % rear right
% initial unsprung mass velocity
uurf=Vsrf(1)-lsirf*wy; 
uulf=Vslf(1)-lsilf*wy; 
uulr=Vslr(1)-lsilr*wy; 
uurr=Vsrr(1)-lsirr*wy; 
vurf=Vsrf(2)+lsirf*wx; 
vulf=Vslf(2)+lsilf*wx; 
vulr=Vslr(2)+lsilr*wx; 
vurr=Vsrr(2)+lsirr*wx;
wulf=w;
wurf=w;
wulr=w;
wurr=w;
% % initial strut acceration
dxslf=-Vslf(3)+wulf;
dxsrf=-Vsrf(3)+wurf;
dxslr=-Vslr(3)+wulr;
dxsrr=-Vsrr(3)+wurr;
% initial unsprung mass acceration
    % derivative of usij & vsij
    dusrf=cf/2*wz_dot+u_dot;
    dvsrf=a*wz_dot+v_dot;
    duslf=-cf/2*wz_dot+u_dot;
    dvslf=a*wz_dot+v_dot;
    duslr=-cr/2*wz_dot+u_dot;
    dvslr=-b*wz_dot+v_dot;
    dusrr=cr/2*wz_dot+u_dot;
    dvsrr=-b*wz_dot+v_dot;
    % acceleration of unprung mass for each corner
    uurf_dot=dusrf-(-dxsrf*wy+lsrf*wy_dot);
    vurf_dot=dvsrf+(-dxsrf*wx+lsrf*wx_dot);
    uulf_dot=duslf-(-dxslf*wy+lslf*wy_dot);
    vulf_dot=dvslf+(-dxslf*wx+lslf*wx_dot);
    uulr_dot=duslr-(-dxslr*wy+lslr*wy_dot);
    vulr_dot=dvslr+(-dxslr*wx+lslr*wx_dot);
    uurr_dot=dusrr-(-dxsrr*wy+lsrr*wy_dot);
    vurr_dot=dvsrr+(-dxsrr*wx+lsrr*wx_dot);
    % wheel rotational velocity
    wlf=u/(r0-xtlf);
    wrf=u/(r0-xtrf);
    wlr=u/(r0-xtlr);
    wrr=u/(r0-xtrr);
%% the tire dynamics module
for i=1:Tsim
% input: state variables uuij,wuij,vuij(for unsprung mass),delta(front wheel steer angle), theta phi psi u,v,w,wx,wy,wz(vehicle body)
% output: tire forces 
ts=(i-1)*0.001:0.0001:i*0.001;
delta=steer_angle(i);
% the instantaneous tire radius
% % to account for the wheel lift-off, when the tire radial compression becomes less than zero, Rij=r0;
if xtrf<0 
    Rrf=r0; 
else
    Rrf=(r0-xtrf)/(cos(theta)*cos(phi));
end
if xtlf<0 
    Rlf=r0;
else
    Rlf=(r0-xtlf)/(cos(theta)*cos(phi));
end
if xtlr<0 
    Rlr=r0; 
else
    Rlr=(r0-xtlr)/(cos(theta)*cos(phi));
end
if xtrr<0 
    Rrr=r0; 
else
    Rrr=(r0-xtrr)/(cos(theta)*cos(phi));
end
% the longitudinal and lateral velocities at the tire contact patch in coordinate frame 2
ugrf=cos(theta)*(uurf-wy*Rrf)+sin(theta)*(wurf*cos(phi)+sin(phi)*(wx*Rrf+vurf));
uglf=cos(theta)*(uulf-wy*Rlf)+sin(theta)*(wulf*cos(phi)+sin(phi)*(wx*Rlf+vulf));
uglr=cos(theta)*(uulr-wy*Rlr)+sin(theta)*(wulr*cos(phi)+sin(phi)*(wx*Rlr+vulr));
ugrr=cos(theta)*(uurr-wy*Rrr)+sin(theta)*(wurr*cos(phi)+sin(phi)*(wx*Rrr+vurr));
vgrf=cos(phi)*(vurf+wx*Rrf)-wurf*sin(phi);
vglf=cos(phi)*(vulf+wx*Rlf)-wulf*sin(phi);
vglr=cos(phi)*(vulr+wx*Rlr)-wulr*sin(phi);
vgrr=cos(phi)*(vurr+wx*Rrr)-wurr*sin(phi);
% vglr=Vslr(2)+wx*(lslr+Rlr);
% vgrr=Vsrr(2)+wx*(lsrr+Rrr);
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
% s_rf=(Rrf*wrf-(ugrf*cos(delta)+vgrf*sin(delta)))/(Rrf*wrf);
s_rf=1-abs((ugrf*cos(delta)+vgrf*sin(delta))/(Rrf*wrf));
else 
% s_rf=(Rrf*wrf-(ugrf*cos(delta)+vgrf*sin(delta)))/(ugrf*cos(delta)+vgrf*sin(delta));
s_rf=abs((Rrf*wrf)/(ugrf*cos(delta)+vgrf*sin(delta)))-1;
end
if Rlf*wlf>(uglf*cos(delta)+vglf*sin(delta))
% s_lf=(Rlf*wlf-(uglf*cos(delta)+vglf*sin(delta)))/(Rlf*wlf);
s_lf=1-abs((uglf*cos(delta)+vglf*sin(delta))/(Rlf*wlf));
else
% s_lf=(Rlf*wlf-(uglf*cos(delta)+vglf*sin(delta)))/(uglf*cos(delta)+vglf*sin(delta));    
s_lf=abs((Rlf*wlf)/(uglf*cos(delta)+vglf*sin(delta)))-1;
end
if Rlr*wlr>uglr
% s_lr=(Rlr*wlr-uglr)/(Rlr*wlr);
s_lr=1-abs(uglr/(Rlr*wlr));
else 
% s_lr=(Rlr*wlr-uglr)/(uglr);
s_lr=abs((Rlr*wlr)/uglr)-1;
end
if Rrr*wrr>ugrr
% s_rr=(Rrr*wrr-ugrr)/(Rrr*wrr);
s_rr=1-abs(ugrr/(Rrr*wrr));
else
% s_rr=(Rrr*wrr-ugrr)/(ugrr);
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
% Fxglr=Fxtlr*cos(delta)-Fytlr*sin(delta);
% Fxgrr=Fxtrr*cos(delta)-Fytrr*sin(delta);
Fyglf=Fxtlf*sin(delta)+Fytlf*cos(delta);
Fygrf=Fxtrf*sin(delta)+Fytrf*cos(delta);
% Fyglr=Fxtlr*sin(delta)+Fytlr*cos(delta);
% Fygrr=Fxtrr*sin(delta)+Fytrr*cos(delta);
Fyglr=Fytlr;
Fygrr=Fytrr;
% to account for the wheel lift-off, when the tire radial compression becomes less than zero, the tire normal force Fz=0
if xtrf<0 
    Fzgrf=0; 
else
    Fzgrf=xtrf*ktf;
end
if xtlf<0 
    Fzglf=0; 
else
    Fzglf=xtlf*ktf;
end
if xtlr<0 
    Fzglr=0; 
else
    Fzglr=xtlr*ktr;
end
if xtrr<0 
    Fzgrr=0; 
else
    Fzgrr=xtrr*ktr;
end
% the tire force in coordinate frame 2
Fglf=[Fxglf;Fyglf;Fzglf];
Fgrf=[Fxgrf;Fygrf;Fzgrf];
Fglr=[Fxglr;Fyglr;Fzglr];
Fgrr=[Fxgrr;Fygrr;Fzgrr];
%rotation matrix
R_y=[cos(theta) 0 -sin(theta);0 1 0;sin(theta) 0 cos(theta)];
R_x=[1 0 0;0 cos(phi) sin(phi);0 -sin(phi) cos(phi)];
% the force acting at the tire ground contact patch in coordinate 1 
Fgslf=R_x*R_y*Fglf;
Fgsrf=R_x*R_y*Fgrf;
Fgslr=R_x*R_y*Fglr;
Fgsrr=R_x*R_y*Fgrr;
%% the wheel rotational modeling
% input: Fxtij(tire forces), Td(driving torque at front wheel), Tb(braking torque at front wheel(Nm))
% output: wlf wrf wlr wrr (wheel rotational velocity)
dwlf=(1/Jw)*(0.5*Ta-0.25*Tb-Fxtlf*Rlf);
dwrf=(1/Jw)*(0.5*Ta-0.25*Tb-Fxtrf*Rrf);
dwlr=(1/Jw)*(-0.25*Tb-Fxtlr*Rlr);
dwrr=(1/Jw)*(-0.25*Tb-Fxtrr*Rrr);  

[t,wlf]=ode45(@(t,wlf) dwlf,ts,wlf);
[t,wrf]=ode45(@(t,wrf) dwrf,ts,wrf);
[t,wlr]=ode45(@(t,wlr) dwlr,ts,wlr);
[t,wrr]=ode45(@(t,wrr) dwrr,ts,wrr);
wlf=wlf(11);
wrf=wrf(11);
wlr=wlr(11);
wrr=wrr(11);

% Fxtlf=((0.5*Ta-0.25*Tb)-u_dot/Rlf*Jw)/Rlf; 
%% the sprung mass dynamics module for each corner
% input: theta phi psi u v w wx wy wz(vehicle body), uuij, vuij, wuij, uuij_dot, vuij_dot, wuij_dot(unsprung mass)
% output: Fxsij Fysij Fzsij Mxij Myij Mzij(the sprung mass forces and moments at each corner)
% the forces transmitted to the sprung mass along x y z directions of coordinate 1
% right front corner
Fxsrf= Fgsrf(1)+muf*g*sin(theta)-muf*uurf_dot+muf*wz*vurf-muf*wy*wurf;
Fysrf= Fgsrf(2)-muf*g*sin(phi)*cos(theta)-muf*vurf_dot+muf*wx*wurf-muf*wz*uurf;
Fzsrf= xsrf*ksrf+dxsrf*bsrf;
% left front corner
Fxslf= Fgslf(1)+muf*g*sin(theta)-muf*uulf_dot+muf*wz*vulf-muf*wy*wulf;
Fyslf= Fgslf(2)-muf*g*sin(phi)*cos(theta)-muf*vulf_dot+muf*wx*wulf-muf*wz*uulf;
Fzslf= xslf*kslf+dxslf*bslf;
% left rear corner
Fxslr= Fgslr(1)+mur*g*sin(theta)-mur*uulr_dot+mur*wz*vulr-mur*wy*wulr;
Fyslr= Fgslr(2)-mur*g*sin(phi)*cos(theta)-mur*vulr_dot+mur*wx*wulr-mur*wz*uulr;
Fzslr= xslr*kslr+dxslr*bslr;
% right rear coner
Fxsrr= Fgsrr(1)+mur*g*sin(theta)-mur*uurr_dot+mur*wz*vurr-mur*wy*wurr;
Fysrr= Fgsrr(2)-mur*g*sin(phi)*cos(theta)-mur*vurr_dot+mur*wx*wurr-mur*wz*uurr;
Fzsrr= xsrr*ksrr+dxsrr*bsrr;
% the force represents the additional load transfer that occurs at the wheels
Fdzrf=(Fgsrf(2)*Rrf+Fysrf*lsrf+Fgslf(2)*Rlf+Fyslf*lslf-(Fysrf+Fyslf)*hrcf)/cf;
Fdzlf=-Fdzrf;
Fdzrr=(Fgsrr(2)*Rrr+Fysrr*lsrr+Fgslr(2)*Rlr+Fyslr*lslr-(Fysrr+Fyslr)*hrcr)/cr;
Fdzlr=-Fdzrr;
% the roll moment transmitted to the sprung mass
Mxrf=Fysrf*hrcf;
Mxlf=Fyslf*hrcf;
Mxlr=Fyslr*hrcr;
Mxrr=Fysrr*hrcr;
% the moments transmitted to the sprung mass by the suspension wy wz directions
Myrf=-(Fgsrf(1)*Rrf+Fxsrf*lsrf);
Mylf=-(Fgslf(1)*Rlf+Fxslf*lslf);
Mylr=-(Fgslr(1)*Rlr+Fxslr*lslr);
Myrr=-(Fgsrr(1)*Rrr+Fxsrr*lsrr);
Mzrf=0;
Mzlf=0;
Mzlr=0;
Mzrr=0;

%% vehicle body dynamics module
% input: Fxsij Fysij Fzsij Mxij Myij Mzij(forces and moments transmitted to sprung mass at each corner)
% output: theta psi phi u v w wx wy wz u_dot v_dot w_dot wx_dot wy_dot wz_dot 
% u_dot=wz*v-wy*w+(1/m)*(Fxslf+Fxsrf+Fxslr+Fxsrr)+g*sin(theta);
u_dot=1/(m+Jw/(Rlf^2)+Jw/(Rrf^2)+Jw/(Rlr^2)+Jw/(Rrr^2))*((0.5*Ta-0.25*Tb)/Rlf+(0.5*Ta-0.25*Tb)/Rrf-0.25*Tb/Rlr-0.25*Tb/Rrr+Fxslf+Fxsrf+Fxslr+Fxsrr+m*wz*v-m*wy*w+m*g*sin(theta));
v_dot=wx*w-wz*u+(1/m)*(Fyslf+Fysrf+Fyslr+Fysrr)-g*sin(phi)*cos(theta);
w_dot=wy*u-wx*v+(1/m)*(Fzslf+Fzsrf+Fzslr+Fzsrr+Fdzlf+Fdzrf+Fdzlr+Fdzrr)-g*cos(phi)*cos(theta);
wx_dot=(1/Jx)*((Mxlf+Mxrf+Mxlr+Mxrr)+(Fzslf-Fzsrf)*cf/2+(Fzslr-Fzsrr)*cr/2);
wy_dot=(1/Jy)*((Mylf+Myrf+Mylr+Myrr)+(Fzslr+Fzsrr)*b-(Fzslf+Fzsrf)*a);
wz_dot=(1/Jz)*((Mzlf+Mzrf+Mzlr+Mzrr)+(Fyslf+Fysrf)*a-(Fyslr+Fysrr)*b+(-Fxslf+Fxsrf)*cf/2 +(-Fxslr+Fxsrr)*cr/2);
% the cardan angles are obtained by performing the integration of the following quations
dtheta=wy*cos(phi)-wz*sin(phi);
dpsi=(wy*sin(phi))/cos(theta)+(wz*cos(phi))/cos(theta);
dphi=wx+wy*sin(phi)*tan(theta)+wz*cos(phi)*tan(theta);
% calculate the position in inertial coordinate
Y_dot=u*sin(psi)+v*cos(psi);
X_dot=u*cos(psi)-v*sin(psi);
% Propagating to time k+1 
[t,u]=ode45(@(t,u) u_dot,ts,u);
[t,v]=ode45(@(t,v) v_dot,ts,v);
[t,w]=ode45(@(t,w) w_dot,ts,w);
[t,wx]=ode45(@(t,wx) wx_dot,ts,wx);
[t,wy]=ode45(@(t,wy) wy_dot,ts,wy);
[t,wz]=ode45(@(t,wz) wz_dot,ts,wz);
[t,theta]=ode45(@(t,theta) dtheta,ts,theta);
[t,psi]=ode45(@(t,psi) dpsi,ts,psi);
[t,phi]=ode45(@(t,phi) dphi,ts,phi);
[t,Y] = ode45(@(t,Y) Y_dot,ts,Y);
[t,X] = ode45(@(t,X) X_dot,ts,X);
u=u(11);
v=v(11);
w=w(11);
wx=wx(11);
wy=wy(11);
wz=wz(11);
theta=theta(11);
psi=psi(11);
phi=phi(11);
Y=Y(11);
X=X(11);
epsi=psi-Xc(12);
ey_dot=u*sin(epsi)+v*cos(epsi);
[t,ey]=ode45(@(t,ey) ey_dot,ts,ey);
ey=ey(11);
%% state update
% velocity of strut mounting point in x y z by transforming the c.g. velocities
wm=[wx;wy;wz];
vm=[u;v;w];
Vsrf=Mrf*wm+vm; % front right mounting point x y z velocity in coordinate frame 1
Vslf=Mlf*wm+vm; % front left
Vslr=Mlr*wm+vm; % rear left
Vsrr=Mrr*wm+vm; % rear right
% the unsprung mass vertical velocity wuij 
dwulf=(1/muf)*(cos(phi)*(cos(theta)*(Fzglf-muf*g)+sin(theta)*Fxglf)-sin(phi)*Fyglf-Fdzlf-xslf*kslf-dxslf*bslf-muf*(vulf*wx-uulf*wy));
dwurf=(1/muf)*(cos(phi)*(cos(theta)*(Fzgrf-muf*g)+sin(theta)*Fxgrf)-sin(phi)*Fygrf-Fdzrf-xsrf*ksrf-dxsrf*bsrf-muf*(vurf*wx-uurf*wy));
dwulr=(1/mur)*(cos(phi)*(cos(theta)*(Fzglr-mur*g)+sin(theta)*Fxglr)-sin(phi)*Fyglr-Fdzlr-xslr*kslr-dxslr*bslr-mur*(vulr*wx-uulr*wy));
dwurr=(1/mur)*(cos(phi)*(cos(theta)*(Fzgrr-mur*g)+sin(theta)*Fxgrr)-sin(phi)*Fygrr-Fdzrr-xsrr*ksrr-dxsrr*bsrr-mur*(vurr*wx-uurr*wy));
[t,wulf]=ode45(@(t,wulf) dwulf,ts,wulf);
[t,wurf]=ode45(@(t,wurf) dwurf,ts,wurf);
[t,wulr]=ode45(@(t,wulr) dwulr,ts,wulr);
[t,wurr]=ode45(@(t,wurr) dwurr,ts,wurr);
wulf=wulf(11);
wurf=wurf(11);
wulr=wulr(11);
wurr=wurr(11);
% the instantaneous suspension spring deflection xs: 
dxslf=-Vslf(3)+wulf;
dxsrf=-Vsrf(3)+wurf;
dxslr=-Vslr(3)+wulr;
dxsrr=-Vsrr(3)+wurr;                
[t,xslf]=ode45(@(t,xslf) dxslf,ts,xslf);
[t,xsrf]=ode45(@(t,xsrf) dxsrf,ts,xsrf);
[t,xslr]=ode45(@(t,xslr) dxslr,ts,xslr);
[t,xsrr]=ode45(@(t,xsrr) dxsrr,ts,xsrr);
xslf=xslf(11);
xsrf=xsrf(11);
xslr=xslr(11);
xsrr=xsrr(11);
% the instantaneous length of the strut
lsrf=lsirf-(xsrf-xsirf);
lslf=lsilf-(xslf-xsilf);
lslr=lsilr-(xslr-xsilr);
lsrr=lsirr-(xsrr-xsirr);
%% the unsprung mass dynamics module for each corner
% input: u v w wx wy wz u_dot v_dot w_dot wx_dot wy_dot wz_dot(vehicle body), Ft(tire force), Fs(sprung force)
% output: uuij, vuij, wuij, uuij_dot, vuij_dot, wuij_dot(unsprung mass)
% derivative of usij & vsij
dusrf=cf/2*wz_dot+u_dot;
dvsrf=a*wz_dot+v_dot;
duslf=-cf/2*wz_dot+u_dot;
dvslf=a*wz_dot+v_dot;
duslr=-cr/2*wz_dot+u_dot;
dvslr=-b*wz_dot+v_dot;
dusrr=cr/2*wz_dot+u_dot;
dvsrr=-b*wz_dot+v_dot;
% acceleration of unprung mass for each corner
uurf_dot=dusrf-(-dxsrf*wy+lsrf*wy_dot);
vurf_dot=dvsrf+(-dxsrf*wx+lsrf*wx_dot);
uulf_dot=duslf-(-dxslf*wy+lslf*wy_dot);
vulf_dot=dvslf+(-dxslf*wx+lslf*wx_dot);
uulr_dot=duslr-(-dxslr*wy+lslr*wy_dot);
vulr_dot=dvslr+(-dxslr*wx+lslr*wx_dot);
uurr_dot=dusrr-(-dxsrr*wy+lsrr*wy_dot);
vurr_dot=dvsrr+(-dxsrr*wx+lsrr*wx_dot);
% the unsprung mass longitudinal and lateral velocities in coordinate frame 1
uurf=Vsrf(1)-lsrf*wy; 
uulf=Vslf(1)-lslf*wy; 
uulr=Vslr(1)-lslr*wy; 
uurr=Vsrr(1)-lsrr*wy; 
vurf=Vsrf(2)+lsrf*wx; 
vulf=Vslf(2)+lslf*wx; 
vulr=Vslr(2)+lslr*wx; 
vurr=Vsrr(2)+lsrr*wx; 
% the instantaneous tire deflection xt: assume the vertical velocity at the tire contact patch is zero (smooth road)
wgrf=0;
wglf=0;
wglr=0;
wgrr=0;
dxtlf=wglf-(cos(theta)*(wulf*cos(phi)+vulf*sin(phi))-uulf*sin(theta));
dxtrf=wgrf-(cos(theta)*(wurf*cos(phi)+vurf*sin(phi))-uurf*sin(theta));
dxtlr=wglr-(cos(theta)*(wulr*cos(phi)+vulr*sin(phi))-uulr*sin(theta));
dxtrr=wgrr-(cos(theta)*(wurr*cos(phi)+vurr*sin(phi))-uurr*sin(theta));
[t,xtlf]=ode45(@(t,xtlf) dxtlf,ts,xtlf);
[t,xtrf]=ode45(@(t,xtrf) dxtrf,ts,xtrf);
[t,xtlr]=ode45(@(t,xtlr) dxtlr,ts,xtlr);
[t,xtrr]=ode45(@(t,xtrr) dxtrr,ts,xtrr);
xtlf=xtlf(11);
xtrf=xtrf(11);
xtlr=xtlr(11);
xtrr=xtrr(11);

% for plot
long_vel(i)=u;
long_acc(i)=u_dot;
roll_angle(i)=phi;
lat_acc(i)=u*wz+v_dot;
yaw_rate(i)=wz;
phi_dot(i)=wx;
lat_vel(i)=v;
psi_angle(i)=psi;
lat_disp(i)=Y;
long_disp(i)=X;
Fzlf(i)=Fzglf;
Fzrf(i)=Fzgrf;
Fzlr(i)=Fzglr;
Fzrr(i)=Fzgrr;
Tlf(i)=xtlf;
Trf(i)=xtrf;
Tlr(i)=xtlr;
Trr(i)=xtrr;
vglf1(i)=vglf;
uglf1(i)=uglf;
vglr1(i)=vglr;
uglr1(i)=uglr;
vgrf1(i)=vgrf;
ugrf1(i)=ugrf;
vgrr1(i)=vgrr;
ugrr1(i)=ugrr;
Vsrf_x(i)=Vsrf(1);
Vsrf_y(i)=Vsrf(2);
Vslf_x(i)=Vslf(1);
Vslf_y(i)=Vslf(2);
Vslr_x(i)=Vslr(1);
Vslr_y(i)=Vslr(2);
Vsrr_x(i)=Vsrr(1);
Vsrr_y(i)=Vsrr(2);
delta_lf1(i)=delta_lf;
delta_rf1(i)=delta_rf;
delta_lr1(i)=delta_lr;
delta_rr1(i)=delta_rr;
end
Ax=long_acc(Tsim);
Ay=lat_acc(Tsim);
Vxo=long_vel(Tsim);
Vy=lat_vel(Tsim);
phi_o=roll_angle(Tsim);
psi=psi_angle(Tsim);
dphi=phi_dot(Tsim);
dpsi=yaw_rate(Tsim);
Y=lat_disp(Tsim);
X=long_disp(Tsim);
xtlf=Tlf(Tsim);
xtrf=Trf(Tsim);
xtlr=Tlr(Tsim);
xtrr=Trr(Tsim); 
Fz_lf=Fzlf(Tsim);
Fz_rf=Fzrf(Tsim);
Fz_lr=Fzlr(Tsim);
Fz_rr=Fzrr(Tsim);
LTR=(Fz_rf+Fz_rr-Fz_lf-Fz_lr)/((m+2*muf+2*mur)*g);
x=[Vxo;Vy;phi_o;psi;dphi;dpsi;Y;X;epsi;ey;wlf;wrf;wlr;wrr;Ax;Ay;v_dot;xtlf;xtrf;xtlr;xtrr;LTR;wz_dot;wx_dot];
