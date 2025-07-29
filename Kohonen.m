function varargout = Kohonen(varargin)
% KOHONEN MATLAB code for Kohonen.fig
%      KOHONEN, by itself, creates a new KOHONEN or raises the existing
%      singleton*.
%
%      H = KOHONEN returns the handle to a new KOHONEN or the handle to
%      the existing singleton*.
%
%      KOHONEN('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in KOHONEN.M with the given input arguments.
%
%      KOHONEN('Property','Value',...) creates a new KOHONEN or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Kohonen_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Kohonen_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Kohonen

% Last Modified by GUIDE v2.5 15-Apr-2019 16:12:55

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Kohonen_OpeningFcn, ...
                   'gui_OutputFcn',  @Kohonen_OutputFcn, ...
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


% --- Executes just before Kohonen is made visible.
function Kohonen_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Kohonen (see VARARGIN)

% Choose default command line output for Kohonen
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
%%%%%%%%%%%%%%%%%%%%%esta funcion ocurre al ejecutar el .m
xlim(handles.axes1,[0 500]) %Define el tamaño de la grafica de la trayectoria
ylim(handles.axes1,[0 500])
xlabel(handles.axes1,'X2');
ylabel(handles.axes1,'X1');

set(handles.slider1,'Value',10)
global u0 t2 NP
u0=0.01; %Inicializacion de constantes para cálculo en el proceso adaptativo
t2=1000;
NP=3; %Número de neuronas por cada punto de la trayectoria (siempre entero, >1)
hold(handles.axes2,'off');
% UIWAIT makes Kohonen wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Kohonen_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles) %%%%Botón "Do"
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pts puntos minX1 maxX1 minX2 maxX2 neuronas W X1 X2 inicio final NP
hold(handles.axes2,'off');
[X2,X1]=getpts(handles.axes1);%obtiene los puntos de las coordenadas de el click del mouse

inicio=[250; 0]; %%Punto inicial
final=[250;500]; %%Punto Final

X2=[0; X2; 500];%%Agrega las coordenadas del punto inicial y final en el eje X2 al vector de valores (Vector Columna)
X1=[250; X1; 250];%%Agrega las coordenadas del punto inicial y final en el eje X1 al vector de valores
pts=[X1 X2]' %%Transpone los datos para organizarlos

hold(handles.axes1,'off') 
plot(handles.axes1,X2,X1,'ro')%%Muestra los puntos de la trayectoria como un circulo rojo
xlim(handles.axes1,[0 500])%Mantiene los limites de la gráfica
ylim(handles.axes1,[0 500])
xlabel(handles.axes1,'X2');
ylabel(handles.axes1,'X1');
hold(handles.axes1,'on') 

puntos=size(pts);
puntos=puntos(2);%Obtiene la cantidad de puntos ingresados
%pts(:,1) Patron 1

neuronas=NP*puntos;%inicializa la cantidad de neuronas

%al inicializar las neuronas de una red de kohonen, estas se ubican dentro
%del rango que se encuentran los datos a procesar, entonces si el menor
%valor en la coordenada X2 de algun dato es, por ejemplo, 10, ninguna
%neurona puede inicializarse en la posicion X2<10; esto aplica para ambas
%coordenadas espaciales.

minX1=min(pts(1,:)); %%Obtiene el rango en el que se encuentran los datos:
minX2=min(pts(2,:));%% minX1<X1<maxX1
maxX1=max(pts(1,:));%% minX2<X2<maxX2
maxX2=max(pts(2,:));

a= minX1 + (maxX1 - minX1).*rand(1,neuronas); %%proceso que genera vectores de ubicaciones aleatorias de las neuronas
b= minX2 + (maxX2 - minX2).*rand(1,neuronas); %%en cada coordenada espacial, dentro del rango definido por min y max
W=[a;b]; %%importante que el formato de 'W' sea compatible con el formato de 'pts'
% W=[x1;x2], pts=[x1;x2], para que no se confundan las coordenadas espaciales
W(:,1)=inicio;%%Toma la primera neurona y hace que su peso (o coordenadas) sea igual a el punto definido inicio,
%esto asegura que la neurona siempre ganara la fase competitiva y sera la
%la primera
W(:,neuronas)=final;%%Lo mismo para la ultima neurona
plot(handles.axes1,W(2,:),W(1,:),'bx')%Plotea las neuronas inicializadas aleatoriamente
xlim(handles.axes1,[0 500])
ylim(handles.axes1,[0 500])
xlabel(handles.axes1,'X2');
ylabel(handles.axes1,'X1');
hold(handles.axes1,'off') 


% --- Executes on button press in togglebutton1.
function togglebutton1_Callback(hObject, eventdata, handles)
% hObject    handle to togglebutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of togglebutton1
global state
state=get(handles.togglebutton1,'Value');%%Botón On/Off, el entrenamiento solo ocurrira si el botón está activado state==true



% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)%%Entrenamiento
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pts puntos neuronas W X2 X1 state skips  t2 u0 %%importante: manejo de las variable globales 

D=zeros(1,neuronas);%%Vector que almacenará las distancias de las neuronas a un punto selecto (Criterio de evaluacion)
sig0=neuronas;%%Sigma0 se inicializa en el número de neuronas (según diapositivas)

c=0;%%Iteraciones para un criterio de parada (1000+(número de neuronas * 500))-->Según literatura (OPCIONAL)
grad=1e-49;%%Cambio que sufrio el total de las neuronas (si es muy pequeño significa que ya paro o no esta
%haciendo ningun significativo)-->Criterio de parada (OPCIONAL)

while state==true && grad > 1e-50 && c< 1000+(neuronas*500)%%}->3 criterios de parada (solo piden el del botón ON/OFF)
    Wv=W;%%Almacenar los pesos antes de variarlos
    sig=sig0*exp(-c/(1000/log(sig0)));%%Formula de sigma (Diapositivas)-->Varía con las iteraciones, para que cada 
                                      %%vez la distibucion normal sea más pequeña
    for k = 1:puntos %%Recorre los puntos
        for i=1:neuronas %%Calcula la distancia de cada neurona al punto seleccionado
            D(1,i)=sum(sum((pts(:,k)-W(:,i)).^2));%%Euclediano
        end
    [~ ,Nmin]=min(D); %%Halla la neurona ganadora (proceso competitivo)
    %%nmin es el número de la neurona que obtuvo la menor distancia al
    %%punto deseado

        for i=1:neuronas%(Proceso cooperativo)

            d=abs(Nmin-i);%%Esta distancia es de la ubicacion de la neurona en el vector, no la distancia euclediana
            h=exp(-(d^2)/(2*(sig^2)));%%Ecuacion de vecindad 

            u=u0*exp(-c/t2);
            if i~=1 && i~=neuronas
            W(:,i)=W(:,i) + (u*h).*(pts(:,k)-W(:,i));%Proceso adaptativo
            end
        end
        if mod(c,skips)==0%%plotea cada "skips" iteraciones
            plot(handles.axes1,X2,X1,'ro')
            hold(handles.axes1,'on') 
            plot(handles.axes1,W(2,:),W(1,:),'bx-')
            xlim(handles.axes1,[0 500])
            ylim(handles.axes1,[0 500])
            xlabel(handles.axes1,'X2');
            ylabel(handles.axes1,'X1');
            drawnow
            hold(handles.axes1,'off') 
            
            plot(handles.axes2,c,grad,'r.');%%Plotea la gradiente por iteracion
            hold(handles.axes2,'on');
            xlim(handles.axes2,[0 c+1]);
            
        end
    end
c=c+1;
grad=sum(sum((W-Wv).^2))%%Calcula el cambio a los pesos

end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
global skips
skips=round(get(handles.slider1,'Value'));%%Slider determina 'Skips'

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double
global NP
NP=round(str2double(get(handles.edit1,'String')));

% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double
global u0
u0=str2double(get(handles.edit2,'String'));

% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
global t2

t2=str2double(get(handles.edit3,'String'));

% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)%%Botón do random
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global pts puntos minX1 maxX1 minX2 maxX2 neuronas W X1 X2 inicio final NP
hold(handles.axes2,'off');
cant=str2double(get(handles.edit5,'String'));

X2= 500.*rand(cant,1); %%proceso que genera vectores de ubicaciones aleatorias de los puntos
X1= 500.*rand(cant,1);

inicio=[250; 0]; %%Punto inicial
final=[250;500]; %%Punto Final

X2=[0; X2; 500];%%Agrega las coordenadas del punto inicial y final en el eje X2 al vector de valores (Vector Columna)
X1=[250; X1; 250];%%Agrega las coordenadas del punto inicial y final en el eje X1 al vector de valores
pts=[X1 X2]' %%Transpone los datos para organizarlos

hold(handles.axes1,'off') 
plot(handles.axes1,X2,X1,'ro')%%Muestra los puntos de la trayectoria como un circulo rojo
xlim(handles.axes1,[0 500])%Mantiene los limites de la gráfica
ylim(handles.axes1,[0 500])
xlabel(handles.axes1,'X2');
ylabel(handles.axes1,'X1');
hold(handles.axes1,'on') 

puntos=size(pts);
puntos=puntos(2);%Obtiene la cantidad de puntos ingresados
%pts(:,1) Patron 1

neuronas=NP*puntos;%inicializa la cantidad de neuronas

%al inicializar las neuronas de una red de kohonen, estas se ubican dentro
%del rango que se encuentran los datos a procesar, entonces si el menor
%valor en la coordenada X2 de algun dato es, por ejemplo, 10, ninguna
%neurona puede inicializarse en la posicion X2<10; esto aplica para ambas
%coordenadas espaciales.

minX1=min(pts(1,:)); %%Obtiene el rango en el que se encuentran los datos:
minX2=min(pts(2,:));%% minX1<X1<maxX1
maxX1=max(pts(1,:));%% minX2<X2<maxX2
maxX2=max(pts(2,:));

a= minX1 + (maxX1 - minX1).*rand(1,neuronas); %%proceso que genera vectores de ubicaciones aleatorias de las neuronas
b= minX2 + (maxX2 - minX2).*rand(1,neuronas); %%en cada coordenada espacial, dentro del rango definido por min y max
W=[a;b]; %%importante que el formato de 'W' sea compatible con el formato de 'pts'
% W=[x1;x2], pts=[x1;x2], para que no se confundan las coordenadas espaciales
W(:,1)=inicio;%%Toma la primera neurona y hace que su peso (o coordenadas) sea igual a el punto definido inicio,
%esto asegura que la neurona siempre ganara la fase competitiva y sera la
%la primera
W(:,neuronas)=final;%%Lo mismo para la ultima neurona
plot(handles.axes1,W(2,:),W(1,:),'bx')%Plotea las neuronas inicializadas aleatoriamente
xlim(handles.axes1,[0 500])
ylim(handles.axes1,[0 500])
xlabel(handles.axes1,'X2');
ylabel(handles.axes1,'X1');
hold(handles.axes1,'off') 



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
