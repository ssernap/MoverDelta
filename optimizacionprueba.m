function T=optimizacionprueba(Th1,Th2,Th3)


Angulos1 =Th1; %angulos objetivos
Angulos2 =Th2;
Angulos3 =Th3;
size=length(Angulos1);
X0=zeros(1,(size-1)/3)+2;   %velocidades Iniciales


%condiciones iniciales
camb=1;     %cambio del tiempo
cont=1;     %contador
alph=0.5;   %correccion del cambio en el gradiente (gradiente*alpha)
delta=0.001;

while(camb>0.001)
    Xtemp=X0;
    for j=1:(length(X0))    
        Vt1 = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt2 = zeros(1,(size-1))+0.0001; %tiempos minimos
        X1=X0; X2=X0;  %inicializa vectores de velocidad
        X1(j)=X1(j)-delta/2; %suma y resta para hacer un delta
        X2(j)=X2(j)+delta/2;
        
        Vt1=SyncronizarTiemposCuartos2(Angulos1,Vt1,X1);%obtiene el vector de tiempos
        Vt2=SyncronizarTiemposCuartos2(Angulos1,Vt2,X2);
        Vt1=SyncronizarTiemposCuartos2(Angulos2,Vt1,X1);%obtiene el vector de tiempos
        Vt2=SyncronizarTiemposCuartos2(Angulos2,Vt2,X2);
        Vt1=SyncronizarTiemposCuartos2(Angulos3,Vt1,X1);%obtiene el vector de tiempos
        Vt2=SyncronizarTiemposCuartos2(Angulos3,Vt2,X2);
               
        t1=Vt1((j-1)*3+1)*0.5+Vt1((j-1)*3+2)+Vt1((j-1)*3+3)*0.5; %tiempoACi*0.5+tiempoVcte+tiempoACf*0.5
        t2=Vt2((j-1)*3+1)*0.5+Vt2((j-1)*3+2)+Vt2((j-1)*3+3)*0.5;

        funp=-(t1-t2)/delta; %derivada
        Xtemp(j)=Xtemp(j)+alph*-funp;
        
    end
    %tiempos finales
    tinicial=zeros(1,(size-1))+0.0001;
    tfinalprev=cumsum(SyncronizarTiemposCuartos2(Angulos1,tinicial,X0));
    T=SyncronizarTiemposCuartos2(Angulos1,tinicial,Xtemp);
    tfinalAct=cumsum(T);
    tfinalprev=tfinalprev(end);
    tfinalAct=tfinalAct(end);
    %actualiza X0
    X0=Xtemp;

    camb=abs(tfinalAct-tfinalprev)/length(X0);

figure(4)
hold all
plot(cont,tfinalAct,'x')

alph=abs((tfinalAct-tfinalprev)/tfinalprev*7)
if(alph>0.5)
    alph=0.4;
end
%alph=alph*0.98;
cont=cont+1;

end

end