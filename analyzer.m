% Cargar datos
data = csvread('robot_data_log.csv', 1, 0); % El 1,0 salta la cabecera

% Asignar variables (columnas)
t = data(:,1)-data(1,1);
rx = data(:,2);
ry = data(:,3);
gx = data(:,5);
gy = data(:,6);
dist_err = data(:,7);
v = data(:,8);

% Ejemplo: Plotear trayectoria realizada vs objetivo
figure;
plot(rx, ry, 'b-', 'LineWidth', 2); hold on;
plot(gx, gy, 'rx');
legend('Trayectoria Robot', 'Goals');
grid on;
title('Trayectoria Pure Pursuit');
xlabel('X [m]'); ylabel('Y [m]');

% Ejemplo: Plotear error de distancia en el tiempo
figure;
plot(t, dist_err, 'r-');
title('Distancia al Goal vs Tiempo');
xlabel('Tiempo [s]'); ylabel('Error [m]');
