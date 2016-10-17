function T2=terciosTiempo(T)
T2=T(1);
for i = 2:length(T)-1
    T2=[T2,T(i)/3,T(i)/3*2];
end

i=length(T);
T2=[T2,T(i)/3,T(i)/3,T(i)/3];
end