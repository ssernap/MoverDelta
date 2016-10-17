function [ThetaM1, ThetaM2, ThetaM3] = ArtiCinInv(Px, Py, Pz,AntThetaM1, AntThetaM2, AntThetaM3)

theta = zeros(3,1);
thetaA = zeros(3,1);
thetaB = zeros(3,1);
alpha = [ 0+pi/2; 2*pi/3+pi/2; 4*pi/3+pi/2];
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


theta(i,1) = atan2( S1, C1 )*2;
thetaA(i,1) = atan2( S1, C1 )*2;
thetaB(i,1) = -atan2( S1, -C1 )*2; 

 

end

if(abs(thetaA(1,1)-AntThetaM1)<abs(thetaB(1,1)-AntThetaM1))
  ThetaM1=thetaA(1,1);  
else
    ThetaM1=thetaB(1,1);  
end


if(abs(thetaA(2,1)-AntThetaM2)<abs(thetaB(2,1)-AntThetaM2))
  ThetaM2=thetaA(2,1);  
else
    ThetaM2=thetaB(2,1);  
end

if(abs(thetaA(3,1)-AntThetaM3)<abs(thetaB(3,1)-AntThetaM3))
  ThetaM3=thetaA(3,1);  
else
    ThetaM3=thetaB(3,1);  
end
