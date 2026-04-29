%Limpieza de pantalla
clear all
close all
clc

%% 1. TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 15;
ts = 0.05;
t = 0:ts:tf;
N = length(t);

%% 2. CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1(1) = 0;
y1(1) = 0;
phi(1) = pi/2;

%% 3. POSICIÓN DESEADA %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

hxd = -5;
hyd = 1;

hx(1) = x1(1);
hy(1) = y1(1);

%% 4. PARÁMETROS CONTROL ADAPTATIVO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Kv0 = 0.8;
Kw0 = 1.5;

alpha_v = 0.08;
alpha_w = 0.05;

% Límites de ganancias
Kv_max = 2.5;
Kw_max = 3.5;

Kv_min = 0.3;
Kw_min = 0.5;

% Límites físicos de velocidad
v_max = 1.2;
w_max = 1.5;

% Variables para suavizado
v_prev = 0;
w_prev = 0;

beta = 0.85; % factor filtro suavizante

%% 5. BUCLE DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:N

    %% a) ERRORES

    hxe(k) = hxd - hx(k);
    hye(k) = hyd - hy(k);

    % Error de distancia
    Error(k) = sqrt(hxe(k)^2 + hye(k)^2);

    % Ángulo deseado
    phi_d = atan2(hye(k), hxe(k));

    % Error angular
    e_phi = phi_d - phi(k);

    % Normalización angular
    e_phi = atan2(sin(e_phi), cos(e_phi));

    %% b) GANANCIAS AUTO-SINTONIZABLES

    Kv(k) = Kv0 + alpha_v*Error(k);
    Kw(k) = Kw0 + alpha_w*abs(e_phi);

    % Saturación de ganancias
    Kv(k) = min(max(Kv(k), Kv_min), Kv_max);
    Kw(k) = min(max(Kw(k), Kw_min), Kw_max);

    %% c) CONTROLADOR

    v_raw = Kv(k)*Error(k);

    % Reduce velocidad si hay mucho error angular
    v_raw = v_raw*cos(e_phi);

    w_raw = Kw(k)*e_phi;

    %% d) SATURACIÓN DE VELOCIDADES

    v_raw = max(min(v_raw, v_max), -v_max);
    w_raw = max(min(w_raw, w_max), -w_max);

    %% e) FILTRO SUAVIZANTE

    v(k) = beta*v_prev + (1-beta)*v_raw;
    w(k) = beta*w_prev + (1-beta)*w_raw;

    v_prev = v(k);
    w_prev = w(k);

    %% f) MODELO CINEMÁTICO

    phi(k+1) = phi(k) + w(k)*ts;

    xp1 = v(k)*cos(phi(k));
    yp1 = v(k)*sin(phi(k));

    x1(k+1) = x1(k) + xp1*ts;
    y1(k+1) = y1(k) + yp1*ts;

    hx(k+1) = x1(k+1);
    hy(k+1) = y1(k+1);

end

%% 6. SIMULACIÓN 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

scene = figure;

set(scene,'Color','white')
set(gca,'FontWeight','bold')

sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);

camlight('headlight');

axis equal
grid on
box on

xlabel('x(m)')
ylabel('y(m)')
zlabel('z(m)')

view([-0.1 35]);

axis([-11 11 -11 11 0 1]);

%% Robot

scale = 4;

MobileRobot_5;
go en 
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale);
hold on;

%% Trayectorias

H2 = plot3(hx(1),hy(1),0,'r','LineWidth',2);

H3 = plot3(hxd,hyd,0,'bo','LineWidth',2);

H4 = plot3(hx(1),hy(1),0,'go','LineWidth',2);

%% Animación

step = 1;

for k = 1:step:N

    delete(H1);
    delete(H2);

    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);

    H2 = plot3(hx(1:k),hy(1:k),zeros(1,k), ...
        'r','LineWidth',2);

    pause(ts);

end

%% 7. GRÁFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

graph = figure;

set(graph,'position',sizeScreen);

subplot(3,1,1)
plot(t,v,'b','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('[m/s]')
legend('Velocidad lineal')

subplot(3,1,2)
plot(t,w,'g','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('[rad/s]')
legend('Velocidad angular')

subplot(3,1,3)
plot(t,Error,'r','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('[m]')
legend('Error')