% Data analyzer
data = csvread('robot_data_log.csv', 1, 0); % El 1,0 salta la cabecera


t = data(:,1)-data(1,1);
rx = data(:,2);
ry = data(:,3);
gx = data(:,5);
gy = data(:,6);
cte = data(:,8);
dist_err = data(:,10);
v = data(:,11);

% Plotear trayectoria realizada vs objetivo
figure;
plot(rx, ry, 'b-', 'LineWidth', 2); hold on;
plot(gx, gy, 'rx');
legend('Trayectoria Robot', 'Goals');
grid on;
title('Trayectoria Pure Pursuit');
xlabel('X [m]'); ylabel('Y [m]');

% Plotear error de distancia en el tiempo
figure;
plot(t, dist_err, 'k-','DisplayName','Error de distancia');hold on
plot(t,0.3*ones(length(t),1),'r','DisplayName','Tolerancia');
title('Distancia al Goal vs Tiempo');
xlabel('Tiempo [s]'); ylabel('Error [m]');legend show
grid on;

% Plotear CTE en el tiempo
figure;
scatter(rx, ry, 20, abs(cte), 'filled'); 
hold on;
plot(gx, gy, 'rx', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
title('Trayectoria del Robot coloreada por CTE (Error Lateral)');
xlabel('X [m]'); 
ylabel('Y [m]');
legend('Trayectoria (Color = Error)', 'Goals');
c = colorbar; 
c.Label.String = 'Error Lateral Absoluto [m]';
colormap jet; 
axis equal;
