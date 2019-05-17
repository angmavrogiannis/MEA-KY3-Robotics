acc=90;
tb=[2,1.66,2,2.11,2.15,3.42].'; %time required for each joint to
vb=acc*tb;                      %reach maxspeed/2
for i=1:6 %timeframe calculation
    qb(i)=q0(i)+0.5*acc*tb(i)^2;
    qc(i)=qf(i)+0.5*acc*tb(i)^2;
    tm(i)=abs(qc(i)-qb(i))/vb(i);
    tf(i)=2*tb(i)+tm(i);
end
a=[1 0 0 300;0 1 0 350;0 0 1 200;0 0 0 1]; %point selection
b=[1 0 0 -350;0 1 0 -250;0 0 1 -300;0 0 0 1];
M76=[1 0 0 0;0 1 0 0;0 0 1 200;0 0 0 1];
A=a/M76;
B=b/M76;
c=zeros(8,1);
Q0=inverse_kinematics(A);
Qf=inverse_kinematics(B);
for j=1:8 %correct selection of the column that represents A and B joint angles
    for i=1:6
        c(j)=c(j)+abs(Qf(i,j)-Q0(i,j));
    end
end
for j=1:8
    if c(j)==min(c)
        n=j;
    end
end
q0=Q0(:,n);
qf=Qf(:,n);
 
%configuration space
s=zeros;
sv=zeros;
for i=1:6 %graphs
    j=1;
    for t=0:tf(i)/100:tf(i)
        if t<tb(i) %1st parabolic
           q=q0(i)+ 45*t^2;
           qv=90*t;
        elseif  t<= (tf(i)-tb(i)) %linear
           q=qb(i)+vb(i)*t;
           qv=vb(i);
        else %2nd parabolic
           q=qc(i)-45*t^2;
           qv=-90*t;
        end
    s(i,j)=q;
    sv(i,j)=qv;
    j=j+1;
    end
end
 
plot(t,s(1,:)) %q1-t graph
title('q1 - t')
xlabel('t')
ylabel('q1')
 
plot(t,sv(1,:)) %dq1/dt-t graph
title('dq1/dt - t')
xlabel('t')
ylabel('dq1/dt')
 
%cartesian space
for j=1:101
    fk=forward_kinematics(s(1,j),s(2,j),s(3,j),s(4,j),s(5,j),s(6,j));
    fk4=fk(:,4);
    for i=1:4 %matrix formation with x,y,z of end effector
        sc(i,j)=fk4(i);
    end 
    r=jacobian(s(:,j))*sv(:,j); 
    for i=1:6
        rd(i,j)=r(i); %velocity dr/dr=J*dq/dt
    end
end
 
t=0:tf(6)/100:tf(6) %z - t graph
figure
plot(t,sc(3,:))
title('z - t')
xlabel('t')
ylabel('z')
 
figure %dz/dt - t graph
plot(t,rd(3,:))
title('dz/dt - t')
xlabel('t')
ylabel('dz/dt')
