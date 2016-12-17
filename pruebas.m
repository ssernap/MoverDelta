Px= Cuartos([1.4473,-0.1920,0.77,0.8751]);
Th = Px;
T1= zeros(1,(length(Px)-1))+0.0001;
Vmax = zeros(1,(length(Px)-1)/3)+2; 
tic
%Vmax=deg2rad(150);%150
Amax=deg2rad(700);%800

% *%Definir tiempo para velocidad maxima en crucero*
contadorVmax=1;
for i=2:3:length(Th)-1 
T1(i)=max(abs(Th(i+1)-Th(i))/Vmax(contadorVmax),T1(i));
contadorVmax=1+contadorVmax;
end

%%velocidades
A=zeros(length(Th)-1,6);

%Calculos de las velocidades constantes
for i=2:3:length(Th)-1 
A(i,:)=[0,0,0,0,(Th(i+1)-Th(i)),Th(i)];    
end

%Calculos de las Velocidades normalizadas intermedias
VelNorm=zeros(1,length(Th));
for i=2:3:length(VelNorm)-1 
    if(i~=2)
        VelNorm(i-1)=(VelNorm(i-1)+A(i,5)/T1(i))/2;
    end
    VelNorm(i)=A(i,5)/T1(i);
    VelNorm(i+1)=A(i,5)/T1(i);
    VelNorm(i+2)=A(i,5)/T1(i);
end
%%endVelocidades




%
% *definir tiempo para aceleracion maxima en polinomio de grado 5*
cambio=1;
T=T1;
cambioT = T+1;
 ciclos =0;
while(cambio>0.001&&ciclos<50) %se sale cuando el tiempo no avance mas de 0.001 segundos
    tiempoInicial=cumsum(T);
ciclos = ciclos+1;
Tant =T;
    %aceleracion
   
for i=1:1:length(T)%recorre los tiempos del polinomio de grado 5    
    
    if(mod(i-2,3)==0)
    else
        if(abs(cambioT(i))>0.000001 )
            if(i==length(T))
                R=[Th(i);VelNorm(i);0;Th(i+1);0;0]; %vector respuesta final
            else
                if(i==1)
                    R=[Th(i);0;0;Th(i+1);VelNorm(i+1);0]; %vector respuesta inicial
                else
                    R=[Th(i);VelNorm(i);0;Th(i+1);VelNorm(i+1);0]; %vector respuesta
                end
            end
   %Matriz de coefficientes
            CC=[  0,  0,  0,  0,   0,   1;
                 0,  0,  0,  0,   1/T(i),   0;  
                 0,  0,  0,  2/T(i)^2,   0,   0;
                 1,  1,  1,  1,   1,   1;
                 5/T(i),  4/T(i),  3/T(i),  2/T(i),   1/T(i),   0;
                20/T(i)^2, 12/T(i)^2,  6/T(i)^2,  2/T(i)^2,   0,   0;
            ];
             C=(CC\R)';
            k=roots(polyder(polyder(polyder(C))));%raices de la derivada de la aceleracion para encontrar maximos y minimos locales
            for j=1:length(k)
                contadorRaiz=0;
                if(k(j)>=0&&k(j)<=1)
                    raiz=sqrt(abs(polyval(polyder(polyder(C))/Amax,k(j))));%aproxmacion del tiempo necesario para T(i)
                    if(isreal(raiz)) 
                        if(contadorRaiz ==0)
                        T(i)=abs(raiz);%max(T(i),abs(raiz));
                        contadorRaiz =2;
                        else
                             T(i)=max(T(i),abs(raiz));                            
                        end
                    
                    end
                end
            end
        end
    end
end

     cambioT=T-Tant;
Tiempofinal=cumsum(T);
cambio=abs(tiempoInicial(end)-Tiempofinal(end)); %cambio total en el tiempo
end
for i=1:1:length(T)
%   T(i)=max(T(i),T1(i)); 
end
toc
CalculoPolinomiosCuartos2(Th,T);