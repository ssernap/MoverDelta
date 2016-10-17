function varargout = moverGUI2(varargin)
% MOVERGUI2 MATLAB code for moverGUI2.fig
%      MOVERGUI2, by itself, creates a new MOVERGUI2 or raises the existing
%      singleton*.
%
%      H = MOVERGUI2 returns the handle to a new MOVERGUI2 or the handle to
%      the existing singleton*.
%
%      MOVERGUI2('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MOVERGUI2.M with the given input arguments.
%
%      MOVERGUI2('Property','Value',...) creates a new MOVERGUI2 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before moverGUI2_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to moverGUI2_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help moverGUI2

% Last Modified by GUIDE v2.5 11-Sep-2016 16:39:14

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @moverGUI2_OpeningFcn, ...
                   'gui_OutputFcn',  @moverGUI2_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before moverGUI2 is made visible.
function moverGUI2_OpeningFcn(hObject, eventdata, handles, varargin)
global coordenadas ingresadas HomeTH1 HomeTH2 HomeTH3
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to moverGUI2 (see VARARGIN)

% Choose default command line output for moverGUI2
handles.output = hObject;
    coordenadas=[0,0,-400,0,0,0,0,0,0];
    HomeTH1 = 0;
    HomeTH2= 0;
    HomeTH3= 0;
    ingresadas = [1]; %este lleva las ingresadas por el ususario
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes moverGUI2 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = moverGUI2_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Comunication_Callback(hObject, eventdata, handles)


function Comunication_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Comunication (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editX_Callback(hObject, eventdata, handles)

function editX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function editZ_Callback(hObject, eventdata, handles)

function editZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function editY_Callback(hObject, eventdata, handles)

function editY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edittiempo_Callback(hObject, eventdata, handles)
% hObject    handle to edittiempo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edittiempo as text
%        str2double(get(hObject,'String')) returns contents of edittiempo as a double


% --- Executes during object creation, after setting all properties.
function edittiempo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edittiempo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonAdd.
function pushbuttonAdd_Callback(hObject, eventdata, handles)
global coordenadas ingresadas
x = double(str2double(get(handles.editX,'String'))); 
y = double(str2double(get(handles.editY,'String'))); 
z = double(str2double(get(handles.editZ,'String'))); 
t = double(str2double(get(handles.edittiempo,'String'))); 
vmax=double(str2double(get(handles.edittiempo,'String'))); 
amax = double(str2double(get(handles.edittiempo,'String'))); 
coordenadas=[coordenadas;
    x, y, z, 0, 0, 0,vmax,amax,t];
ingresadas=[ingresadas,1]

h = handles.uitable1; %(outTable1 is the Tag of my uitable)
%cnames = get(h,'ColumnName');
set(h,'Data',coordenadas);




% --- Executes on button press in pushbuttonConnect.
function pushbuttonConnect_Callback(hObject, eventdata, handles)
global COM ID1 ID2


%%lee las librerias
loadlibrary('dynamixel','dynamixel.h');
libfunctions('dynamixel');

%%reviza la conexion
BAUDNUM = 1;
%COM = int32(str2double(get(handles.Comunication,'String'))); 
COM=0;
%%carga los ID y el puerto
ID1=2;
ID2=1;
%ID3=(str2double(get(handles.IDMotor3,'String')));
conectado =0;

while(conectado==0)
COM=COM+1;
  Prueba_Coneccion = calllib('dynamixel','dxl_initialize',COM,BAUDNUM);
%   
    if (COM>50)
        conectado=1;
    end
  if (Prueba_Coneccion==0)
       set(handles.conexion,'String','NO CONECTADO')
       set(handles.conexion,'ForegroundColor',[1 0 0]);
  else
        set(handles.conexion,'String','SI CONECTADO')
       set(handles.conexion,'ForegroundColor',[0 1 0]);
       set(handles.Comunication,'String',int2str(COM))
       conectado =1;
  end
end
% --- Executes on button press in pushbuttonHome.
function pushbuttonHome_Callback(hObject, eventdata, handles)
    global HomeTH1 HomeTH2 HomeTH3
     RegPosActual=36;
     aa=Deg2Digital(180);
     HomeTH1=-Digital2Deg(calllib('dynamixel','dxl_read_word',1,RegPosActual));% el -Deg2Digital(180) centra el home
     HomeTH2=-Digital2Deg(calllib('dynamixel','dxl_read_word',2,RegPosActual));
    
    %HomeTH3=int32(calllib('dynamixel','dxl_read_word',ID3,RegPosActual))-Deg2Digital(180);
    
    
% hObject    handle to pushbuttonHome (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% ---------------------------------------------------------------------------------- Executes on button press in pushbutton2Angle.
function pushbutton2Angle_Callback(hObject, eventdata, handles)
global coordenadas ingresadas Tiempo A1 A2 A3
Px1temp=  coordenadas(:,1);
Py1temp=  coordenadas(:,2);
Pz1temp=  coordenadas(:,3);

Px1=Px1temp(1);
Py1=Py1temp(1);
Pz1=Pz1temp(1);
for i=2:length(Px1temp)

    if(ingresadas(i)==1)
    Px1=[Px1,Px1temp(i)];
    Py1=[Py1,Py1temp(i)];
    Pz1=[Pz1,Pz1temp(i)];
    end
    
end
ingretemp= [1];
contadorIngresadas=0;
for i=1:length(ingresadas)
   if(ingresadas(i)==1)
   contadorIngresadas = contadorIngresadas+1;
   end
end

for i=2:contadorIngresadas
   if(i==(contadorIngresadas))
    ingretemp= [ingretemp,0,0,1]; 
   else
       ingretemp= [ingretemp,0,1]; 
   end
end
ingresadas =ingretemp; 


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



%syncronizar  buscar tiempos minimos
Tiempo=SyncronizarTiempos(Th1,Tiempo);
Tiempo=SyncronizarTiempos(Th2,Tiempo);
Tiempo=SyncronizarTiempos(Th3,Tiempo);
coordenadas=[Px',Py',Pz',Th1',Th2',Th3',Px',Px',[0,Tiempo]']

h = handles.uitable1; %(outTable1 is the Tag of my uitable)
%cnames = get(h,'ColumnName');
set(h,'Data',coordenadas);


A1=CalculoPolinomios(Th1,Tiempo);
A2=CalculoPolinomios(Th2,Tiempo);
A3=CalculoPolinomios(Th3,Tiempo);

 TiempoAcu=[0,cumsum(Tiempo)]
% TiempoSimu=floor(TiempoAcu(end)*1000)/1000;


% --- Executes on button press in pushbuttonExecute.
function pushbuttonExecute_Callback(hObject, eventdata, handles)
% hObject    handle to pushbuttonExecute (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbuttonDisconnect.
function pushbuttonDisconnect_Callback(hObject, eventdata, handles)
calllib('dynamixel','dxl_terminate');
%Se libera la libreria y el control de puerto del servo motor.
unloadlibrary('dynamixel');
set(handles.conexion,'ForegroundColor',[0 0 0]);
set(handles.conexion,'String','NO CONECTADO')







% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbuttonExecute.
function pushbuttonExecute_ButtonDownFcn(hObject, eventdata, handles)


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over pushbuttonConnect.
function pushbuttonConnect_ButtonDownFcn(hObject, eventdata, handles)


% --- Executes on button press in pushbuttonpasarAngoptim.
function pushbuttonpasarAngoptim_Callback(hObject, eventdata, handles)
global coordenadas ingresadas Tiempo A1 A2 A3
Px1temp=  coordenadas(:,1);
Py1temp=  coordenadas(:,2);
Pz1temp=  coordenadas(:,3);
TiempoTemp=coordenadas(:,end);
Px1=Px1temp(1);
Py1=Py1temp(1);
Pz1=Pz1temp(1);
Tiempo=TiempoTemp(1)
for i=2:length(Px1temp)

    if(ingresadas(i)==1)
    Px1=[Px1,Px1temp(i)];
    Py1=[Py1,Py1temp(i)];
    Pz1=[Pz1,Pz1temp(i)];
    Tiempo=[Tiempo,TiempoTemp(i)];
    end
    
end
ingretemp= [1];
contadorIngresadas=0;
for i=1:length(ingresadas)
   if(ingresadas(i)==1)
   contadorIngresadas = contadorIngresadas+1;
   end
end

for i=2:contadorIngresadas
   if(i==(contadorIngresadas))
    ingretemp= [ingretemp,0,0,1]; 
   else
       ingretemp= [ingretemp,0,1]; 
   end
end
ingresadas =ingretemp; 


% Calculo de valores intermedios y tiempo inicial
Px= tercios(Px1);
Pz= tercios(Pz1);
Py= tercios(Py1);
Tiempo= terciosTiempo(Tiempo);


%calculo de angulos
[Th1(1),Th2(1), Th3(1)] =ArtiCinInv(Px(1), Py(1), Pz(1),0,0, 0);
for i=2:length(Px)
     [Th1(i),Th2(i), Th3(i)] =ArtiCinInv(Px(i), Py(i), Pz(i),Th1(i-1),Th2(i-1), Th3(i-1));
end



%syncronizar  buscar tiempos minimos
 Amax=deg2rad(800);
%[Tiempo,amax1]=SyncronizarTiempos2(Th1,Tiempo,Amax+5);
%[Tiempo,amax2]=SyncronizarTiempos2(Th2,Tiempo,Amax+5);
%[Tiempo,amax3]=SyncronizarTiempos2(Th3,Tiempo,Amax+5);
coordenadas=[Px',Py',Pz',Th1',Th2',Th3',Px',Px',Tiempo']

h = handles.uitable1; %(outTable1 is the Tag of my uitable)
%cnames = get(h,'ColumnName');
set(h,'Data',coordenadas);


A1=CalculoPolinomios(Th1,Tiempo);
A2=CalculoPolinomios(Th2,Tiempo);
A3=CalculoPolinomios(Th3,Tiempo);
Tiempo=Tiempo(2:end);
 TiempoAcu=[0,cumsum(Tiempo)]
% TiempoSimu=floor(TiempoAcu(end)*1000)/1000;






            
            
 function SyncWrite(motorPositions)
% tic
		%MX64.syncWritePosition Set the position of multiple consecutive actuators simultaneously.

            
			motorCount = length(motorPositions);                            %numero de motores

            calllib('dynamixel','dxl_set_txpacket_id',254);					% Broadcast ID (fijo)
            calllib('dynamixel','dxl_set_txpacket_instruction',131);		% Sync Command (fijo)
            calllib('dynamixel','dxl_set_txpacket_parameter',0,30);	        % Primera direccion de escritura (en este caso posicion baja 30)

            paramNumber = 3;                                                % Numero de parametros por motor->3 (ID, Posicion baja, Posicion Alta)%para el for
            calllib('dynamixel','dxl_set_txpacket_parameter',1,paramNumber - 1);    	%Cantidad de Parametros en un motor->2 (Posicion baja, Posicion Alta)
			% loop over each motor position given
			for id = 1:motorCount
                angle = motorPositions(id); %obtiene el angulo objetivo del motor

                calllib('dynamixel','dxl_set_txpacket_parameter',(3*id)-1,id);          % ID of motor we want to address
                                                                                        %3*id-1 es el numero del parametro
                lowByte = calllib('dynamixel','dxl_get_lowbyte',angle);                 % calculate low byte (LSB) of 2 byte word
                highByte = calllib('dynamixel','dxl_get_highbyte',angle);               % calculate high byte (MSB) of 2 byte word
                calllib('dynamixel','dxl_set_txpacket_parameter',3*id,lowByte);         % write low byte
                calllib('dynamixel','dxl_set_txpacket_parameter',(3*id)+1,highByte);    % write high byte
            end

            % length = (L+1) X N + 4 | L = length of subPacket, N = number of servos
            calllib('dynamixel','dxl_set_txpacket_length',(paramNumber*motorCount)+4);  % Tama;o del paquete total
            calllib('dynamixel','dxl_txrx_packet');                                     % Transmit packet
%             toc
            
          


% --- Executes on button press in pushbuttonGraficar.
function pushbuttonGraficar_Callback(hObject, eventdata, handles)
global Tiempo A1 A2 A3
    
    TiempoAcu =[0,cumsum(Tiempo)];
i=1;
delta=0.01;
for t=1:5000
    Time(t) = t/5000*TiempoAcu(end);
while (Time(t)>TiempoAcu(i))
    i=i+1;
end
i=i-1;
if(i==0)
    i=1;
end
DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));
%evalua los polinomios     
 ThetaM1(t)=polyval(A1(i,:),(Time(t)-TiempoAcu(i))/DesNorm);
 ThetaM2(t)=polyval(A2(i,:),(Time(t)-TiempoAcu(i))/DesNorm);
 ThetaM3(t)=polyval(A3(i,:),(Time(t)-TiempoAcu(i))/DesNorm);
 
end
 TiempoAcu(end)
figure(1)
plot(Time,[rad2deg(ThetaM1)])
hold all
plot(Time,[rad2deg(ThetaM2)])
plot(Time,[rad2deg(ThetaM3)])
grid on

%  ThPM1=polyval(polyder(A1(i,:))/Tiempo(i),(Time-TiempoAcu(i))/DesNorm);
%  ThPM2=polyval(polyder(A2(i,:))/Tiempo(i),(Time-TiempoAcu(i))/DesNorm);
%  ThPM3=-polyval(polyder(A3(i,:))/Tiempo(i),(Time-TiempoAcu(i))/DesNorm);
%  
%  ThPPM1=polyval(polyder(polyder(A1(i,:)))/Tiempo(i)^2,(Time-TiempoAcu(i))/DesNorm);
%  ThPPM2=polyval(polyder(polyder(A2(i,:)))/Tiempo(i)^2,(Time-TiempoAcu(i))/DesNorm);
%  ThPPM3=-polyval(polyder(polyder(A3(i,:)))/Tiempo(i)^2,(Time-TiempoAcu(i))/DesNorm);
% 


% --- Executes on button press in pushbuttonMover.
function pushbuttonMover_Callback(hObject, eventdata, handles)
global Tiempo A1 A2 A3
global HomeTH1 HomeTH2 HomeTH3
TiempoAcu =[0,cumsum(Tiempo)];
i=1;    


% ir a la posicion inicial
DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));
 ThetaM1=polyval(A1(i,:),(0)/DesNorm);
 ThetaM2=polyval(A2(i,:),(0)/DesNorm);
 ThetaM3=-polyval(A3(i,:),(0)/DesNorm);
 
 b=Deg2Digital(rad2deg(ThetaM1)-HomeTH1-90);
 c=Deg2Digital(rad2deg(ThetaM2)-HomeTH2-90);
 SyncWrite([b,c]);
pause(3)
%



tic
tfinal=TiempoAcu(end);
t=0;

Time = toc;
while Time<tfinal
    t=t+1;
     a(t)=toc;
    
while (Time>TiempoAcu(i))
    i=i+1;
end
i=i-1;
if(i==0)
    i=1;
end
DesNorm=(TiempoAcu(i+1)-TiempoAcu(i));
%evalua los polinomios     
 ThetaM1=polyval(A1(i,:),(Time-TiempoAcu(i))/DesNorm);
 ThetaM2=polyval(A2(i,:),(Time-TiempoAcu(i))/DesNorm);
 ThetaM3=polyval(A3(i,:),(Time-TiempoAcu(i))/DesNorm);
 
 b=Deg2Digital(rad2deg(ThetaM1)-HomeTH1-90);
 c=Deg2Digital(rad2deg(ThetaM2)-HomeTH2-90);
 SyncWrite([b,c]);
 while(toc-Time<0.0012)
   
 end
 Time = toc;
end

figure(1);
% plot(a);
% hold all
% figure(2)
 plot(diff(a));
% figure(3)
% plot(b,c);

grid on

function deg=Digital2Deg (digital)
deg=digital*360.0/4095.0-180;


function digital=Deg2Digital(angulo)
angulo=angulo+180; %centrar al motor, yo lo tengo centrado en 0
    
    
if(angulo>360)
    angulo=359;
end
if(angulo<0)
    angulo=1;
end

digital = int32(angulo*4095.0/360.0);



function editP_Callback(hObject, eventdata, handles)
% hObject    handle to editP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editP as text
%        str2double(get(hObject,'String')) returns contents of editP as a double


% --- Executes during object creation, after setting all properties.
function editP_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editP (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editI_Callback(hObject, eventdata, handles)
% hObject    handle to editI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editI as text
%        str2double(get(hObject,'String')) returns contents of editI as a double


% --- Executes during object creation, after setting all properties.
function editI_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editI (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editD_Callback(hObject, eventdata, handles)
% hObject    handle to editD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editD as text
%        str2double(get(hObject,'String')) returns contents of editD as a double


% --- Executes during object creation, after setting all properties.
function editD_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editV_Callback(hObject, eventdata, handles)
% hObject    handle to editV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editV as text
%        str2double(get(hObject,'String')) returns contents of editV as a double


% --- Executes during object creation, after setting all properties.
function editV_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editV (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function editTorque_Callback(hObject, eventdata, handles)
% hObject    handle to editTorque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of editTorque as text
%        str2double(get(hObject,'String')) returns contents of editTorque as a double


% --- Executes during object creation, after setting all properties.
function editTorque_CreateFcn(hObject, eventdata, handles)
% hObject    handle to editTorque (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbuttonGoHOME.
function pushbuttonGoHOME_Callback(hObject, eventdata, handles)

    
    
    
    
    
    