function Th2=Cuartos(Th)
Th2=Th(1);
% for i = 2:length(Th)-1
%     Th2=[Th2,(2*Th(i-1)+Th(i))/3,(Th(i-1)+2*Th(i))/3,Th(i)];
% end
% i=length(Th);
% Th2=[Th2,(2*Th(i-1)+Th(i))/3,(Th(i-1)+2*Th(i))/3,Th(i)];


a=3;
b=a-1;
for i = 2:length(Th)-1
    Th2=[Th2,(b*Th(i-1)+Th(i))/a,(Th(i-1)+b*Th(i))/a,Th(i)];
end
i=length(Th);
Th2=[Th2,(b*Th(i-1)+Th(i))/a,(Th(i-1)+b*Th(i))/a,Th(i)];



end