function [ThetaM1, ThetaM2, ThetaM3] = ArtiCinInv(Px, Py, Pz,AntThetaM1, AntThetaM2, AntThetaM3)

%theta = zeros(3,1);
thetaCodo1 = zeros(3,1);
thetaCodo2 = zeros(3,1);

alpha = [ 0+pi/2; 2*pi/3+pi/2; 4*pi/3+pi/2]; %angulos de rotacion de los motoresm(0, 120, 240)+offset
R  = 65.25; % 34.5
r  = 34.47;% 65.5;
L1  = 130; % 130;
L2  = 275; % 275;

R1=R-r;
Si=1/L1*(-Px^2-Py^2-Pz^2+L2^2-L1^2-R1^2);
for i=1:3
Qi=2*Px*cos(alpha(i))+2*Py*sin(alpha(i));
S1=-2*Pz-sqrt(4*Pz^2+4*R1^2-Si^2+Qi^2*(1-(R1^2)/(L1^2))+Qi*((-2*R1*Si)/L1-4*R1));
C1=-2*R1-Qi*(R1/L1-1)-Si;


%theta(i,1) = atan2( S1, C1 )*2;
thetaCodo1(i,1) = atan2( S1, C1 )*2;
thetaCodo2(i,1) = -atan2( S1, -C1 )*2; 

 

end

if(abs(thetaCodo1(1,1)-AntThetaM1)<abs(thetaCodo2(1,1)-AntThetaM1))
  ThetaM1=thetaCodo1(1,1);  
else
    ThetaM1=thetaCodo2(1,1);  
end


if(abs(thetaCodo1(2,1)-AntThetaM2)<abs(thetaCodo2(2,1)-AntThetaM2))
  ThetaM2=thetaCodo1(2,1);  
else
    ThetaM2=thetaCodo2(2,1);  
end

if(abs(thetaCodo1(3,1)-AntThetaM3)<abs(thetaCodo2(3,1)-AntThetaM3))
  ThetaM3=thetaCodo1(3,1);  
else
    ThetaM3=thetaCodo2(3,1);  
end
