function U = invkin(M60)

L2=315;
L3=35;
d4=365;
U=zeros(6,8);
P4x=M60(1,4);
P4y=M60(2,4);
P4z=M60(3,4);
if P4x==0
    if P4y==0
        disp('Error,singularity!')
    else
        u11=90;
        u12=-90;
    end
elseif P4y==0
    u11=0;
    u12=180;
else
    u11=atan2d(P4y,P4x);
    u12=atan2d(-P4y,-P4x);
end
U1=[u11,u12];
a=2*L2*L3;
b=2*L2*d4;
C1=[cosd(u11),cosd(u11),cosd(u12),cosd(u12)];
ci=(P4x/C1(1))^2+P4z^2-L3^2-L2^2-d4^2;
cii=(P4x/C1(2))^2+P4z^2-L3^2-L2^2-d4^2;
u311=atan2d(b,a)+atan2d(sqrt(a^2+b^2-ci^2),ci);
u312=atan2d(b,a)-atan2d(sqrt(a^2+b^2-ci^2),ci);
u321=atan2d(b,a)+atan2d(sqrt(a^2+b^2-cii^2),cii);
u322=atan2d(b,a)-atan2d(sqrt(a^2+b^2-cii^2),cii);
U3=[u311,u312,u321,u322];
SCU3=[sind(u311),sind(u312),sind(u321),sind(u322);cosd(u311),cosd(u312),cosd(u321),cosd(u322)];
for i = 1:4
    F=L3*SCU3(2,i)+d4*SCU3(1,i)+L2;
    G=L3*SCU3(1,i)-d4*SCU3(2,i);
    x=P4x/C1(i);
    U2(i)=atan2d((F*P4z-G*x),(F*x+G*P4z));
end
u211=U2(1);
u212=U2(2);
u221=U2(3);
u222=U2(4);
for i = 1:2
    u1=U1(i);
    for j = 1:4
        
        if i == 1
            U(1,j)=u11;
            if j<3
                U(2,j)=U2(1);
                U(3,j)=U3(1);
            else
                U(2,j)=U2(2);
                U(3,j)=U3(2);
            end
        else
            U(1,4+j)=u12;
            if j<3
                U(2,4+j)=U2(3);
                U(3,4+j)=U3(3);
            else
                U(2,4+j)=U2(4);
                U(3,4+j)=U3(4);
            end
        end
        u3=U3(j);
        u2=U2(j);
         M10=[cosd(U(1,j)) -sind(U(1,j)) 0 0;sind(U(1,j)) cosd(U(1,j)) 0 0;0 0 1 0;0 0 0 1];
         M21=[cosd(U(2,j)) -sind(U(2,j)) 0 0;0 0 -1 0;sind(U(2,j)) cosd(U(2,j)) 0 0;0 0 0 1];
        M32=[cosd(U(3,j)) -sind(U(3,j)) 0 L2;sind(U(3,j)) cosd(U(3,j)) 0 0;0 0 1 0;0 0 0 1];
        M30=M10*M21*M32;
        R63=inv(M30)*M60;
        n1=-R63(2,3);
        if i==1
            if mod(j,2)==1
                U(5,j)=atan2d(sqrt(1-n1^2),n1);
                n21=R63(2,1)/sind(U(5,j));
                n31=R63(2,2)/(-sind(U(5,j)));
                U(6,j)=atan2d(n31,n21);
                n41=R63(1,3)/sind(U(5,j));
                n51=R63(3,3)/sind(U(5,j));
                U(4,j)=atan2d(n51,n41);
            else
                U(5,j)=atan2d(-sqrt(1-n1^2),n1);
                n22=R63(2,1)/sind(U(5,j));
                n32=R63(2,2)/(-sind(U(5,j)));
                U(6,j)=atan2d(n32,n22);
                n42=R63(1,3)/sind(U(5,j));
                n52=R63(3,3)/sind(U(5,j));
                U(4,j)=atan2d(n52,n42);
            end
        else
            if mod(j,2)==1
                U(5,j+4)=atan2d(sqrt(1-n1^2),n1);
                n21=R63(2,1)/sind(U(5,j+4));
                n31=R63(2,2)/(-sind(U(5,j+4)));
                U(6,j+4)=atan2d(n31,n21);
                n41=R63(1,3)/sind(U(5,j+4));
                n51=R63(3,3)/sind(U(5,j+4));
                U(4,j+4)=atan2d(n51,n41);
            else
                U(5,j+4)=atan2d(-sqrt(1-n1^2),n1);
                n22=R63(2,1)/sind(U(5,j+4));
                n32=R63(2,2)/(-sind(U(5,j+4)));
                U(6,j+4)=atan2d(n32,n22);
                n42=R63(1,3)/sind(U(j+4));
                n52=R63(3,3)/sind(U(j+4));
                U(4,j+4)=atan2d(n52,n42);
            end
        end
    end
end
disp(U)