%% Interpolador de orden 5 en el espacio articular, con tiempo dado
%%Sebastian Serna Palleja
%%ssernap@unal.edu.co
%%Version 2
%%Abril 2016
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function A=PreCalcTh(Th,Tiempo)
                        
    TiempoAcu=[0,cumsum(Tiempo)];       %Vector con acumulado del tiempo
    Por=0.75;                           %porcentaje maximo de la Vmax
    VMaxAbs=5*Por;                      %Velocidad maxima
    AcMax=0;                            %Aceleracion Maxima

    SB(1)=1;                            %Guarda las posiciones de Cambios entre subida y bajada
    cont=1;                             %Contador , lleva la cantidad de cambios
    %detecta subidas y bajadas
    for i=2:length(Th)-1
        if (Th(i-1)-Th(i))*(Th(i+1)-Th(i))<0

        else
            cont=cont+1;
            SB(cont)=i;
        end
    end
    SB(cont+1)=length(Th);

    %crea las V iniciales y las Ac iniciales
    for i=1:cont 
        ThP(SB(i))=0;   %fronteras Vel en 0
        ThPP(SB(i))=0;  %fronteras Ac en 0

        for j=SB(i)+1:SB(i+1)-1
                % _Distancia entre los dos thetas_
            Dist=Th(SB(i)+1)-Th(SB(i));
                % _Velocidad maxima calculada a partir del tiempo y la distancia_ 
            VMax=(Th(SB(i)+1)-Th(SB(i)))/(TiempoAcu(SB(i)+1)-TiempoAcu(SB(i)));
                % _Vector con las velocidades, asemejandose a la funcion seno_
            ThP(j)=abs(VMax*sin((Dist*(pi()))/(Th(SB(i+1))-Th(SB(i)))))*sign((Th(SB(i+1))-Th(SB(i))));
                % _Vector con las aceleraciones, asemejandose a la funcion seno_
            ThPP(j)=((Th(j+1)-Th(j))/Tiempo(j)-(Th(j)-Th(j-1))/Tiempo(j))/((Tiempo(j)+Tiempo(j-1))/2);  
    %       ThPP(j)=AcMax*sin((Dist*(2*pi()))/(Th(k(i+1))-Th(k(i)))); 

        end

    end
    ThP(length(Th))=0;   %fronteras Vel
    ThPP(length(Th))=0;  %fronteras Ac




    
    %calcula polinomios
    for i=1:length(Th)-1
    R=[Th(i);ThP(i);ThPP(i);Th(i+1);ThP(i+1);ThPP(i+1)]; %vector respuesta
      %  a    b   c   d   e   f
    M=[  0,  0,  0,  0,   0,   1;
         0,  0,  0,  0,   1,   0;  
         0,  0,  0,  2,   0,   0;
         1,  1,  1,  1,   1,   1;
         5,  4,  3,  2,   1,   0;
        20, 12,  6,  2,   0,   0;
    ];
if(i==1)
A=(inv(M)*R)';
else
A=[A;(inv(M)*R)'];    
end
    
%     a(i)=A(1);
%     b(i)=A(2);
%     c(i)=A(3);
%     d(i)=A(4);
%     e(i)=A(5);
%     f(i)=A(6);
    end

    end