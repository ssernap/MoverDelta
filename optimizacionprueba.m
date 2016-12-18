function T=optimizacionprueba(Th1,Th2,Th3)


Angulos1 =Th1; %angulos objetivos
Angulos2 =Th2;
Angulos3 =Th3;
size=length(Angulos1);
X0a=zeros(1,(size-1)/3)+2;   %velocidades Iniciales
X0b=zeros(1,(size-1)/3)+2;   %velocidades Iniciales
X0c=zeros(1,(size-1)/3)+2;   %velocidades Iniciales


%condiciones iniciales
camb=1;     %cambio del tiempo
cont=1;     %contador
alph=0.05;   %correccion del cambio en el gradiente (gradiente*alpha)
delta=0.005;

while(camb>0.001)
    Xtempa=X0a;
    Xtempb=X0b;
    Xtempc=X0c;
    for j= 1:(length(X0a))    
        Vt1a = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt2a = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt1b = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt2b = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt1c = zeros(1,(size-1))+0.0001; %tiempos minimos
        Vt2c = zeros(1,(size-1))+0.0001; %tiempos minimos
        
        X1a=Xtempa; X2a=Xtempa;  %inicializa vectores de velocidad
        X1a(j)=X1a(j)-delta/2; %suma y resta para hacer un delta
        X2a(j)=X2a(j)+delta/2;
        X1b=Xtempb; X2b=Xtempb;  %inicializa vectores de velocidad
        X1b(j)=X1b(j)-delta/2; %suma y resta para hacer un delta
        X2b(j)=X2b(j)+delta/2;
        X1c=Xtempc; X2c=Xtempc;  %inicializa vectores de velocidad
        X1c(j)=X1c(j)-delta/2; %suma y resta para hacer un delta
        X2c(j)=X2c(j)+delta/2;
        
        
        
        
        Vt1a=SyncronizarTiemposCuartos2(Angulos1,Vt1a,X1a);%obtiene el vector de tiempos
        Vt2a=SyncronizarTiemposCuartos2(Angulos1,Vt2a,X2a);
        Vt1b=SyncronizarTiemposCuartos2(Angulos2,Vt1b,X1b);%obtiene el vector de tiempos
        Vt2b=SyncronizarTiemposCuartos2(Angulos2,Vt2b,X2b);
        Vt1c=SyncronizarTiemposCuartos2(Angulos3,Vt1c,X1c);%obtiene el vector de tiempos
        Vt2c=SyncronizarTiemposCuartos2(Angulos3,Vt2c,X2c);
               
        t1a=cumsum(Vt1a); %tiempoACi*0.5+tiempoVcte+tiempoACf*0.5
        t2a=cumsum(Vt2a);
        t1a=t1a(end);
        t2a=t2a(end);
        funpa=-(t1a-t2a)/delta; %derivada
        if(abs(Xtempa(j)*0.1)>abs(alph*-funpa))
           mover = alph*-funpa;
        else
            mover =sign(alph*-funpa)*Xtempa(j)*0.1;
        end
        Xtempa(j)=Xtempa(j)+mover; 
        
                t1b=cumsum(Vt1b); %tiempoACi*0.5+tiempoVcte+tiempoACf*0.5
        t2b=cumsum(Vt2b);
        t1b=t1b(end);
        t2b=t2b(end);
        funpb=-(t1b-t2b)/delta; %derivada
        
        if(abs(Xtempb(j)*0.1)>abs(alph*-funpb))
           mover = alph*-funpb;
        else
            mover =sign(alph*-funpb)*Xtempb(j)*0.1;
        end
        Xtempb(j)=Xtempb(j)+mover; 
        
                t1c=cumsum(Vt1c); %tiempoACi*0.5+tiempoVcte+tiempoACf*0.5
        t2c=cumsum(Vt2c);
        t1c=t1c(end);
        t2c=t2c(end);
        funpc=-(t1c-t2c)/delta; %derivada
        
            if(abs(Xtempc(j)*0.1)>abs(alph*-funpc))
           mover = alph*-funpc;
        else
            mover =sign(alph*-funpc)*Xtempc(j)*0.1;
        end
        Xtempc(j)=Xtempc(j)+mover; 
        
        
       
        
    end
    %tiempos finales
    tinicial=zeros(1,(size-1))+0.0001;
   % tfinalprev=cumsum(SyncronizarTiemposCuartos2(Angulos1,tinicial,X0a));    
    tfinalprev=max(SyncronizarTiemposCuartos2(Angulos1,tinicial,X0a),SyncronizarTiemposCuartos2(Angulos2,tinicial,X0b));
    tfinalprev = max (tfinalprev,SyncronizarTiemposCuartos2(Angulos3,tinicial,X0c)); 
    tfinalprev = cumsum(tfinalprev);
   
    
    T=max(SyncronizarTiemposCuartos2(Angulos1,tinicial,Xtempa),SyncronizarTiemposCuartos2(Angulos2,tinicial,Xtempb));
    T = max (T,SyncronizarTiemposCuartos2(Angulos3,tinicial,Xtempc));
    tfinalAct=cumsum(T);
    tfinalprev=tfinalprev(end);
    tfinalAct=tfinalAct(end);
    %actualiza X0
   
    camb=abs(tfinalAct-tfinalprev)/length(X0a);
    if((tfinalAct-tfinalprev)/tfinalAct<0.03||cont==1)
    X0a=Xtempa;
    X0b=Xtempb;
    X0c=Xtempc;    
    figure(4)
    hold all
    plot(cont,tfinalAct,'o','color','b')
delta=0.005;
    alph=abs((tfinalAct-tfinalprev)/tfinalprev*7)
    if(alph>0.5)
        alph=0.4*0.95^cont;;
    end
    if(alph<0.1)
        alph=0.1*0.95^cont;
    end
    %alph=alph*0.98;
    cont=cont+1;
    
    else
        %cont = cont -1;
        delta=0.001;
         alph =  alph *0.5;
          plot(cont,tfinalAct,'o','color','g')
    end



end

end



% 
% on T=optimizacionprueba(Th1,Th2,Th3)
% 
% 
% Angulos1 =Th1; %angulos objetivos
% Angulos2 =Th2;
% Angulos3 =Th3;
% size=length(Angulos1);
% X0=zeros(1,(size-1)/3)+2;   %velocidades Iniciales
% 
% 
% %condiciones iniciales
% camb=1;     %cambio del tiempo
% cont=1;     %contador
% alph=0.05;   %correccion del cambio en el gradiente (gradiente*alpha)
% delta=0.005;
% 
% while(camb>0.001)
%     Xtemp=X0;
%     for j= 1:(length(X0))    
%         Vt1 = zeros(1,(size-1))+0.0001; %tiempos minimos
%         Vt2 = zeros(1,(size-1))+0.0001; %tiempos minimos
%         X1=Xtemp; X2=Xtemp;  %inicializa vectores de velocidad
%         X1(j)=X1(j)-delta/2; %suma y resta para hacer un delta
%         X2(j)=X2(j)+delta/2;
%         
%         Vt1=SyncronizarTiemposCuartos2(Angulos1,Vt1,X1);%obtiene el vector de tiempos
%         Vt2=SyncronizarTiemposCuartos2(Angulos1,Vt2,X2);
%     %    Vt1=SyncronizarTiemposCuartos2(Angulos2,Vt1,X1);%obtiene el vector de tiempos
%      %   Vt2=SyncronizarTiemposCuartos2(Angulos2,Vt2,X2);
%      %   Vt1=SyncronizarTiemposCuartos2(Angulos3,Vt1,X1);%obtiene el vector de tiempos
%      %   Vt2=SyncronizarTiemposCuartos2(Angulos3,Vt2,X2);
%                
%         t1=cumsum(Vt1); %tiempoACi*0.5+tiempoVcte+tiempoACf*0.5
%         t2=cumsum(Vt2);
%         t1=t1(end);
%         t2=t2(end);
%         
%         funp=-(t1-t2)/delta; %derivada
%         Xtemp(j)=Xtemp(j)+alph*-funp;
%         
%     end
%     %tiempos finales
%     tinicial=zeros(1,(size-1))+0.0001;
%     tfinalprev=cumsum(SyncronizarTiemposCuartos2(Angulos1,tinicial,X0));
%     T=SyncronizarTiemposCuartos2(Angulos1,tinicial,Xtemp);
%     tfinalAct=cumsum(T);
%     tfinalprev=tfinalprev(end);
%     tfinalAct=tfinalAct(end);
%     %actualiza X0
%     X0=Xtemp;
% 
%     camb=abs(tfinalAct-tfinalprev)/length(X0);
% 
% figure(4)
% hold all
% plot(cont,tfinalAct,'x')
% 
% alph=abs((tfinalAct-tfinalprev)/tfinalprev*7)
% if(alph>0.5)
%     alph=0.4;
% end
% if(alph<0.1)
%     alph=0.1*0.95^cont;
% end
% %alph=alph*0.98;
% cont=cont+1;
% 
% end
% 
% end