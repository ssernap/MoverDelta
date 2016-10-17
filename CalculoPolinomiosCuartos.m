function A=CalculoPolinomiosCuartos(Th,T)

A=zeros(length(Th)-1,6);

%Calculos de las velocidades constantes
for i=2:3:length(Th)-1 
A(i,:)=[0,0,0,0,(Th(i+1)-Th(i)),Th(i)];    
end

%Calculos de las Velocidades intermedias
Velocidades=zeros(1,length(Th));
for i=2:3:length(Velocidades)-1 
    if(i~=2)
        Velocidades(i-1)=(Velocidades(i-1)+A(i,5))/2;
    end
    Velocidades(i)=A(i,5);
    Velocidades(i+1)=A(i,5);
    Velocidades(i+2)=A(i,5);
end



for i = 1:1:length(Th)-1 
    if(mod(i-2,3)==0) %velocidad Constante
    else %aceleracion
    
    
   if(i==length(Th)-1 )
   R=[Th(i);Velocidades(i);0;Th(i+1);0;0]; %vector respuesta FINAL
   else
       if(i==1)
           R=[Th(i);0;0;Th(i+1);Velocidades(i+1);0]; %vector respuesta INICIAL
       else
           R=[Th(i);Velocidades(i);0;Th(i+1);Velocidades(i+1);0]; %vector respuesta
       end
   end


      %  a    b   c   d   e   f
    M=[  0,  0,  0,  0,   0,   1;
         0,  0,  0,  0,   1/T(i),   0;  
         0,  0,  0,  2/T(i)^2,   0,   0;
         1,  1,  1,  1,   1,   1;
         5/T(i),  4/T(i),  3/T(i),  2/T(i),   1/T(i),   0;
        20/T(i)^2, 12/T(i)^2,  6/T(i)^2,  2/T(i)^2,   0,   0;
    ];




    A(i,:)=[(inv(M)*R)'];  

    
    end
end



TiempoAcu= [0,cumsum(T)];
for Cont=1:floor(TiempoAcu(end)*100)
    Time=Cont/100;
    i=1;
    while (Time>TiempoAcu(i))
        i=i+1;
    end
    i=i-1;
    if(i==0)
        i=1;
    end
    
    
    
        DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));    
        Tiempo(Cont)=Time;
        ThetaM1(Cont)=polyval(A(i,:),(Time-TiempoAcu(i))/DesNorm);
        ThetaM1P(Cont)=polyval(polyder(A(i,:)/DesNorm),(Time-TiempoAcu(i))/DesNorm);
        ThetaM1PP(Cont)=polyval(polyder(polyder(A(i,:)))/T(i)^2,(Time-TiempoAcu(i))/DesNorm);
end
figure(1)
plot(Tiempo,ThetaM1);
figure(2)
plot(Tiempo,ThetaM1P);
figure(3)
plot(Tiempo,ThetaM1PP);

end

    
    
    
    
    