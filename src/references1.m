function [obj,arc_l,s_l]=references1
% clc;
% clear;
%% reference trajectory generation
% 1. read path points data from file (X Y is with respect to arc length) 
pathpoints_arc=readtable('pathpoints_arc.xlsx');
X0=pathpoints_arc(:,1);
Y0=pathpoints_arc(:,2);
X0=cell2mat(table2cell(X0));
Y0=cell2mat(table2cell(Y0));

% 2. generate points along the trajectory using spline curve
arc_l=length(X0)-1;
s=arc_l; % s-the length of the trajectory;
st=0:1:arc_l;
i=0;
ds=0.2;
s_l=ds;
for ss=0:ds:arc_l
   i=i+1;
   Xp(i)=interp1(st,X0,ss,'spline');
   Yp(i)=interp1(st,Y0,ss,'spline');
end
% ss1=0:ds:arc_l;
% figure;plot(ss1,Xp);hold on;plot(st,X0,'r--');
% figure;plot(ss1,Yp);hold on;plot(st,Y0,'r--');
% figure;plot(Xp,Yp);hold on;plot(X0,Y0,'r--');

%3. calculate the curvature, approximated using finite differences of the sample waypoints 
for i=1:length(Xp)-1
    dX(i)=(Xp(i+1)-Xp(i))/ds;
    dY(i)=(Yp(i+1)-Yp(i))/ds;  
end
for i=1:length(Xp)-2
    ddX(i)=(Xp(i+2)-2*Xp(i+1)+Xp(i))/ds^2;
    ddY(i)=(Yp(i+2)-2*Yp(i+1)+Yp(i))/ds^2;
end
for i=1:length(Xp)-2
K(i)=(dX(i)*ddY(i)-dY(i)*ddX(i))/(dX(i)^2+dY(i)^2)^1.5;
end

% calculate the reference heading angle 
darc=0:ds:s-2*ds;
Psi0=atan((Yp(2)-Yp(1))/((Xp(2)-Xp(1))));
Psi=Psi0+cumtrapz(darc,K);

% calculate the reference X position
Fx=cos(Psi);
X1=Xp(1)+cumtrapz(darc,Fx);

% calculate the reference Y position
Fy=sin(Psi);
Y1=Yp(1)+cumtrapz(darc,Fy);

% generate the reference longitudinal velocity
mu=0.9;
g=9.81;
for i=1:length(Xp)-2
    if K(i)==0
        V=35;
    else
        vx=sqrt(0.4*mu*g/abs(K(i)));
        if vx>35
            V=35;
        else
            V=vx;   
        end
    end
    Vx(i)=V;
end
m=1720;
Rmax=500;
i=0;
for sarc=0:ds:s-2*ds
    i=i+1;
    if K(i)==0
        R=Rmax;
    else
        R=min(1/(abs(K(i))),Rmax);
    end
    Ra(i)=R;
end

i=0;
for sarc=0:ds:s-2*ds
    i=i+1;
    sy=cal_ay(Vx(i),Ra(i));
    ay=sy(1);
    Fz=sy(2);
    Fy=m*ay;
    Fx=sqrt((mu*Fz)^2-Fy^2);
    Fxacc(i)=Fx;
end
Vp1(1)=Vx(1);
for i=1:length(Vx)-1
    Vp1(i+1)=sqrt(Vp1(i)^2+2*Fxacc(i+1)/m*ds);
    if Vp1(i+1)<Vx(i+1)
       Vp1(i+1)=Vp1(i+1);
    else
       Vp1(i+1)=Vx(i+1);
    end
end
Vp2=zeros(1,length(Vx));
Vp2(length(Vx))=Vp1(length(Vx));

Fxdec=Fxacc;
for i=length(Vx):-1:2
    Vp2(i-1)=sqrt(Vp2(i)^2+2*Fxdec(i-1)/m*ds);
    if Vp2(i-1)<Vp1(i-1)
        Vp2(i-1)=Vp2(i-1);
    else
        Vp2(i-1)=Vp1(i-1);
    end
end    
Vp2a=Vp2;
Xref=X1;
Yref=Y1;
Pref=Psi;
Kref=K;

%9. set the velocity is increasing from Vp0 with acceleration of axf 
% Vp0=0.5029;
% axf=mu*g;
% Vp=Vp0;
% Vf=Vp2(1);
% Vp3(1)=Vp0;
% S1=20;
% i=1;
% for l=0:ds:S1
%     X=l;
%     if Vp<Vf  
%        Vp=sqrt(Vp0^2+2*ds*axf);   
%        Vp3(i+1)=Vp;
%        S1x=X;
%     else
%        Vp3(i+1)=Vp2(i); 
%     end  
%     Vp0=Vp;
%     i=i+1;
%     Vf=Vp2(i);
% end
% i=0;
% for l=0:ds:S1x
%     i=i+1;
%     Vp2a(i)=Vp3(i);
% end
Vref=Vp2a;

% calculate the acceleration
Ax(1)=(Vref(2)^2-Vref(1)^2)/(2*ds);
for i=1:length(Vref)-1
    ax=(Vref(i+1)^2-Vref(i)^2)/(2*ds);
    Ax(i+1)=ax;
end

for i=1:length(Vref)
    Xref1(i)=Xref(i);
    Yref1(i)=Yref(i);
end

obj=[Xref1;Yref1;Pref;Kref;Vref;Ax;Vp1;Vp2;Vx;Fxacc];