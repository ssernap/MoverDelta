function T=SyncronizarTiempos(Th,T)
Vmax=deg2rad(150);%150
Amax=deg2rad(800);%800

% *%Definir tiempo para velocidad maxima en crucero*
for i=2:2:length(Th)-1
T(i)=max(abs(Th(i+1)-Th(i))/Vmax,T(i));
end

%
% *definir tiempo para aceleracion maxima en polinomio de grado 5*
cambio=1;
while(cambio>0.001) %se sale cuando el tiempo no avance mas de 0.001 segundos
    tiempoInicial=cumsum(T);

   
    %aceleracion
for i=1:2:length(T)%recorre los tiempos del polinomio de grado 5    
    if(i==length(T))
   R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);0;0]; %vector respuesta
   else
       if(i==1)
           R=[Th(i);0;0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
       else
           R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
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
     C=(inv(CC)*R)';
     
    k=roots(polyder(polyder(polyder(C))));%raices de la derivada de la aceleracion para encontrar maximos y minimos locales
    for j=1:length(k)
        if(k(j)>=0&&k(j)<=1)
        raiz=sqrt(abs(polyval(polyder(polyder(C))/Amax,k(j))));%aproxmacion del tiempo necesario para T(i)
            if(isreal(raiz))    
                T(i)=max(T(i),abs(raiz));
            end
        end

    end

end
Tiempofinal=cumsum(T);
cambio=abs(tiempoInicial(end)-Tiempofinal(end)); %cambio total en el tiempo
end



% cambio=1;
% while(cambio>0.001) %se sale cuando el tiempo no avance mas de 0.001 segundos
%     tiempoInicial=cumsum(T);
% 
%   
%     %velocidad
% for i=1:2:length(T)%recorre los tiempos del polinomio de grado 5    
%     if(i==length(T))
%    R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);0;0]; %vector respuesta
%    else
%        if(i==1)
%            R=[Th(i);0;0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
%        else
%            R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
%        end
%     end
%    %Matriz de coefficientes
%     CC=[  0,  0,  0,  0,   0,   1;
%          0,  0,  0,  0,   1/T(i),   0;  
%          0,  0,  0,  2/T(i)^2,   0,   0;
%          1,  1,  1,  1,   1,   1;
%          5/T(i),  4/T(i),  3/T(i),  2/T(i),   1/T(i),   0;
%         20/T(i)^2, 12/T(i)^2,  6/T(i)^2,  2/T(i)^2,   0,   0;
%     ];
%      C=(inv(CC)*R)';
%      
%     k=roots((polyder(polyder(C))));%raices de la derivada de la aceleracion para encontrar maximos y minimos locales
%     for j=1:length(k)
%         if(k(j)>=0&&k(j)<=1)
%         raiz=(abs(polyval((polyder(C))/Vmax,k(j))));%aproxmacion del tiempo necesario para T(i)
%             if(isreal(raiz))    
%                 T(i)=max(T(i),abs(raiz));
%             end
%         end
% 
%     end
% 
% end
% Tiempofinal=cumsum(T);
% cambio=abs(tiempoInicial(end)-Tiempofinal(end)); %cambio total en el tiempo
% end




end