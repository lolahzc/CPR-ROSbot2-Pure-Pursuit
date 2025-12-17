files = dir('*.csv'); % Busca todos los CSV

if isempty(files)
    error('No hay archivos CSV en la carpeta.');
else
    % Ordenar por fecha de modificación (descendente)
    [~, idx] = sort([files.datenum], 'descend');
    most_recent_file = files(idx(1)).name;
    
    fullFileName = fullfile(pwd, most_recent_file);
    disp(['Cargando automáticamente el último experimento: ', most_recent_file]);
    
    try
        data = csvread(fullFileName, 1, 0);
    catch
        error('Error leyendo el archivo.');
    end
end

% ---------------------------------------------------------------------------

t = data(:,1)-data(1,1);
rx = data(:,2);
ry = data(:,3);
gx = data(:,5);
gy = data(:,6);
ld = data(:,7);
cte = data(:,8);
dist_err = data(:,10);
v = data(:,11);

% -------------------------------------------------------------------------------

% Plotear trayectoria realizada vs objetivo
figure;
plot(rx, ry, 'b-', 'LineWidth', 2); hold on;
plot(gx, gy, 'rx');
legend('Trayectoria Robot', 'Goals');
grid on;
title('Trayectoria Pure Pursuit');
xlabel('X [m]'); ylabel('Y [m]');

% -------------------------------------------------------------------------------


% Plotear error de distancia en el tiempo
figure;
plot(t, dist_err, 'k-','DisplayName','Distancia a Waypoint');hold on
plot(t,0.3*ones(length(t),1),'r','DisplayName','Tolerancia');
title('Distancia al Goal vs Tiempo');
xlabel('Tiempo [s]'); ylabel('Distancia [m]');legend show
grid on;

% -------------------------------------------------------------------------------

%% Comparativa Temporal (Causa - Efecto)
figure; 
umbral_val = 0.04; 
logic_vector = abs(cte) > umbral_val;
indices_cruces = find(diff(logic_vector) != 0);
tiempos_cruces = t(indices_cruces);

% Subplot 1
ax1 = subplot(2,1,1);
plot(t, abs(cte), 'r-', 'LineWidth', 1.5); hold on;
plot(t, umbral_val * ones(size(t)), 'k--', 'LineWidth', 1.2);
text(t(1), umbral_val * 1.1, 'Umbral 0.05m', 'FontSize', 10, 'Color', 'black');

y_limits_1 = ylim;
for i = 1:length(tiempos_cruces)
    tc = tiempos_cruces(i);
    plot([tc tc], y_limits_1, 'g--', 'LineWidth', 1);
end

grid on;
ylabel('|CTE| [m]');
title('Magnitud del Error Lateral (CTE)');
legend('Error Actual', 'Umbral', 'Cruce de Umbral');
hold off;

% Subplot 2
ax2 = subplot(2,1,2);hold on;
plot(t, v, 'b-', 'LineWidth', 1.5);

y_limits_2 = ylim; % Limites verticales de la gráfica de velocidad
for i = 1:length(tiempos_cruces)
    tc = tiempos_cruces(i);
    plot([tc, tc], y_limits_2, 'g--', 'LineWidth', 1);
end

grid on;
xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
title('Velocidad Lineal del Robot');

linkaxes([ax1, ax2], 'x');

% -------------------------------------------------------------------------------

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
%c.Label.String = 'Error Lateral Absoluto [m]';
colormap jet; 
axis equal;

% -------------------------------------------------------------------------------
drawnow; 

disp(' ');
disp('=============================================================');
disp('GRÁFICAS GENERADAS CORRECTAMENTE');
disp('Octave está en pausa para mantener las ventanas abiertas.');
disp('PRESIONA [ENTER] EN ESTA TERMINAL PARA CERRAR Y SALIR.');
disp('=============================================================');

input('', 's'); 

close all; % Cierra todas las ventanas gráficas ordenadamente
disp('Terminando sesión de análisis.');

