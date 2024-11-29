clc;
clear;
close all;

% Inicializar la cámara
cam = webcam; % Selecciona la cámara predeterminada
frameRate = 30; % Ajusta el frame rate aproximado de la cámara

% Parámetros de color para el seguimiento de la pelota
u_cb = 70; % valor medio para el canal Cb
u_cr = 133; % valor medio para el canal Cr
sigma_cb = 30; % desviación estándar para el canal Cb
sigma_cr = 30; % desviación estándar para el canal Cr

% Parámetros del filtro de Kalman
dt = 1/frameRate; % intervalo de tiempo basado en el frame rate
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % matriz de transición de estado
H = [1 0 0 0; 0 1 0 0]; % matriz de observación
Q = diag([0.1, 0.1, 1, 1]); % Ruido del proceso ajustado
R = diag([10, 10]); % Ruido de la medición ajustado
P = eye(4); % Covarianza inicial
alpha = 0.6; % Factor de suavizado

% Leer el primer cuadro de la cámara para inicializar
im1 = snapshot(cam);
im2 = snapshot(cam);
FSM = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr);
[min_col_1, min_row_1, max_col, max_row, widthbbox, highbbox] = calculateROI(FSM);

if isempty(min_col_1) || isempty(min_row_1)
    error('No se detectó ninguna región en los primeros cuadros. Asegúrate de que la pelota esté visible.');
end

% Inicializar posición
xt = [min_col_1; min_row_1; 0; 0];
xtp = xt;

% Crear ventana de visualización
figure;
hImage = imshow(im1);
hold on;
hRectangle = rectangle('Position', [xt(1), xt(2), widthbbox, highbbox], 'EdgeColor', 'red', 'LineWidth', 2);

% Procesamiento en tiempo real
while ishandle(hImage)
    % Capturar cuadro actual
    im1 = im2; % El cuadro anterior
    im2 = snapshot(cam); % Cuadro actual

    % Detección de la pelota
    FSM = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr);
    [min_col, min_row, max_col, max_row, widthbbox, highbbox] = calculateROI(FSM);

    % Verificación si la pelota fue detectada
    if ~isempty(min_col) && ~isempty(min_row)
        % Si la pelota es detectada, actualizar con observación real
        zt = [min_col; min_row];
        K = P * H' / (H * P * H' + R);  % Ganancia de Kalman

        % Corrección en posición
        xt(1:2) = xt(1:2) + K(1:2, :) * (zt - H * xt);

        % Suavizado de la posición estimada
        xt(1:2) = alpha * xt(1:2) + (1 - alpha) * zt;

        % Actualización de la covarianza
        P = (eye(4) - K * H) * P;
    else
        % Si no se detecta, solo predecir sin corrección
        K = zeros(size(P * H'));
    end

    % Predicción del siguiente estado
    xt = A * xt;
    P = A * P * A' + Q;

    % Actualizar el cuadro y el rectángulo en la figura
    set(hImage, 'CData', im2);
    set(hRectangle, 'Position', [xt(1), xt(2), widthbbox, highbbox]);
    drawnow;

    % Romper el loop con la tecla ESC
    if ~ishandle(hImage)
        break;
    end
end

% Liberar la cámara
clear cam;

%% Función de segmentación del color de la pelota amarilla
function [SM] = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr)
    % Convertir imagen a espacio YCbCr
    YCBCR = rgb2ycbcr(im1);
    Cb = YCBCR(:,:,2);
    Cr = YCBCR(:,:,3);

    % Segmentación basada en el color amarillo
    St = (Cb > (u_cb - sigma_cb)) & (Cb < (u_cb + sigma_cb)) & ...
         (Cr > (u_cr - sigma_cr)) & (Cr < (u_cr + sigma_cr));

    % Segmentación de movimiento entre fotogramas
    Fdt = abs(im2 - im1);
    Fdg = 0.299 * Fdt(:,:,1) + 0.587 * Fdt(:,:,2) + 0.114 * Fdt(:,:,3);
    T = 0.05 * mean(Fdg, 'all');
    Fdb = imbinarize(Fdg, T);

    % Región de interés final
    SM = St & Fdb;
    SM = medfilt2(SM);
end

%% Función para calcular la región de interés (ROI)
function [min_col, min_row, max_col, max_row, w, h] = calculateROI(SM)
    [rows, cols] = find(SM);
    if isempty(rows) || isempty(cols)
        min_col = [];
        max_col = [];
        min_row = [];
        max_row = [];
        w = 0;
        h = 0;
    else
        % Añadir margen al ROI
        margin = 5; % Ajustar según necesidad
        min_col = max(1, min(cols) - margin);
        max_col = min(size(SM, 2), max(cols) + margin);
        min_row = max(1, min(rows) - margin);
        max_row = min(size(SM, 1), max(rows) + margin);
        w = max_col - min_col;
        h = max_row - min_row;
    end
end
