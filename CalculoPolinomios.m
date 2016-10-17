function A=CalculoPolinomios(Th,T)


for i = 1:2:length(Th)-1 
   if(i==length(Th)-1 )
   R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);0;0]; %vector respuesta
   else
       if(i==1)
           R=[Th(i);0;0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
       else
           R=[Th(i);(Th(i)-Th(i-1))/T(i-1);0;Th(i+1);(Th(i+2)-Th(i+1))/T(i+1);0]; %vector respuesta
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

    if(i==1)
    A=(inv(M)*R)';
    else
    A=[A;(inv(M)*R)'];  

    end



    if(i==length(Th)-1 )
    else
    A=[A;
        0,0,0,0,(Th(i+2)-Th(i+1)),Th(i+1)];
    end
   
    

end




% for Cont=1:floor(TiempoAcu(end)*100)
%     Time=Cont/100;
%     i=1;
%     while (Time>TiempoAcu(i))
%         i=i+1;
%     end
%     i=i-1;
%     if(i==0)
%         i=1;
%     end
%     
%     
%     
%         DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));    
%         ThetaM1(Cont)=polyval(A(i,:),(Time-TiempoAcu(i))/DesNorm);
%         ThetaM1P(Cont)=polyval(polyder(A(i,:)/DesNorm),(Time-TiempoAcu(i))/DesNorm);
%         ThetaM1PP(Cont)=polyval(polyder(polyder(A(i,:)))/T(i)^2,(Time-TiempoAcu(i))/DesNorm);
% end

end

    
    
    
    
    