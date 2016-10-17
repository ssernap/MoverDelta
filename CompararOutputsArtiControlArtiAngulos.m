%load('C:\Users\Sebastian\Desktop\Tesis B\T_SM\Outputs\ComparacionArti.mat');
figure(1)
plot(salida2.time,salida2.signals.values(:,1),'color','r','linewidth',2)
hold all
plot(salida1.time,salida1.signals.values(:,1),'color','b','linewidth',2)
grid on
%plot(TiempoAcu,Py,'o');


figure(2)
plot(salida2.time,salida2.signals.values(:,2),'color','r','linewidth',2)
hold all
plot(salida1.time,salida1.signals.values(:,2),'color','b','linewidth',2)
grid on
%plot(TiempoAcu,Px,'o');



figure(3)
plot(salida2.time,salida2.signals.values(:,3),'color','r','linewidth',2)%salida de angulos
hold all
plot(salida1.time,salida1.signals.values(:,3),'color','b','linewidth',2)%salida de control
grid on

%plot(TiempoAcu,Pz,'o');
figure(4)
for i=1:length(salida1.time)
    
    k(i)=(salida1.signals.values(i,3))-buscar(salida2.time',salida2.signals.values(:,3)',salida1.time(i));

end
hold all
plot(salida1.time,k,'b','linewidth',2);
grid on
xlabel('Tiempo(s)');
ylabel('error en z (mm)');