Px1=  [  -70,   -70,   70,  70  ];
Pz1=  [-300, -160,- 160, -300];
Py1=  [     0,      0,     0,      0];


% Calculo de valores intermedios y tiempo inicial
Px= tercios(Px1);
Pz= tercios(Pz1);
Py= tercios(Py1);
Tiempo=(1:length(Px)-1)*0+0.1;

%calculo de angulos
[Th1(1),Th2(1), Th3(1)] =ArtiCinInv(Px(1), Py(1), Pz(1),0,0, 0);
for i=2:length(Px)
     [Th1(i),Th2(i), Th3(i)] =ArtiCinInv(Px(i), Py(i), Pz(i),Th1(i-1),Th2(i-1), Th3(i-1));
end



%syncronizar
Tiempo=SyncronizarTiempos(Th1,Tiempo);
Tiempo=SyncronizarTiempos(Th2,Tiempo);
Tiempo=SyncronizarTiempos(Th3,Tiempo);

A1=CalculoPolinomios(Th1,Tiempo);
A2=CalculoPolinomios(Th2,Tiempo);
A3=CalculoPolinomios(Th3,Tiempo);



A=A3;
TiempoAcu=[0,cumsum(Tiempo)];
T=Tiempo;
for Cont=1:floor(TiempoAcu(end)*100)
    Time=Cont/100;
    Time2(Cont)=Cont/100;
    i=1;
    while (Time>TiempoAcu(i))
        i=i+1;
    end
    i=i-1;
    if(i==0)
        i=1;
    end
    
    
    
        DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));    
        ThetaM1(Cont)=polyval(A(i,:),(Time-TiempoAcu(i))/DesNorm);
        ThetaM1P(Cont)=polyval(polyder(A(i,:)/DesNorm),(Time-TiempoAcu(i))/DesNorm);
        ThetaM1PP(Cont)=polyval(polyder(polyder(A(i,:)))/T(i)^2,(Time-TiempoAcu(i))/DesNorm);
end
plot(Time2,ThetaM1)