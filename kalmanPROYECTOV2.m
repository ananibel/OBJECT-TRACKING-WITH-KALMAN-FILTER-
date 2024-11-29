clc; 
close all;
clear;

% Inicializar video
video_name = 'pelota_rebote_1D_comprimido.mp4';
vid = VideoReader(video_name); 
frameRate = vid.FrameRate;
nframes = vid.NumFrames; 
Height = vid.Height;
Width = vid.Width;

% Parámetros de color para el seguimiento de la pelota
u_cb = 70; % valor medio para el canal Cb
u_cr = 133; % valor medio para el canal Cr
sigma_cb = 30; % desviación estándar para el canal Cb
sigma_cr = 30; % desviación estándar para el canal Cr

% Parámetros del filtro de Kalman
dt = 1; % intervalo de tiempo
A = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1]; % matriz de transición de estado
H = [1 0 0 0; 0 1 0 0]; % matriz de observación
Q = diag([0.1, 0.1, 1, 1]); % Ruido del proceso ajustado
R = diag([10, 10]); % Ruido de la medición ajustado
P = eye(4); % Covarianza inicial
alpha = 0.6; % Factor de suavizado

% Posición inicial
im1 = read(vid,1);
im2 = read(vid,2);
FSM = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr);
[min_col_1, min_row_1, max_col, max_row, widthbbox, highbbox] = calculateROI(FSM); 

if isempty(min_col_1) || isempty(min_row_1)
    error('No se detectó ninguna región en los primeros cuadros. Verifica los parámetros de color y asegúrate de que la pelota sea visible.');
end

im1 = read(vid,2); 
im2 = read(vid,3);
FSM = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr);
[min_col, min_row, ~, ~, ~, ~] = calculateROI(FSM); 

if isempty(min_col) || isempty(min_row)
    error('No se detectó ninguna región en los cuadros iniciales. Verifica los parámetros de color y asegúrate de que la pelota sea visible.');
end

% Definir vector de estado inicial como vector de 4x1
xt = [min_col; min_row; (min_col - min_col_1)/dt; (min_row - min_row_1)/dt];
xtp = xt;

% Procesamiento de cada fotograma para el seguimiento
for k = 3:nframes-1
    im1 = read(vid,k); 
    im2 = read(vid,k+1);
    
    % Detección de la pelota
    FSM = getBallPosition(im1, im2, u_cb, u_cr, sigma_cb, sigma_cr);
    [min_col, min_row, max_col, max_row, widthbbox, highbbox] = calculateROI(FSM); 

    % Verificación si la pelota fue detectada
    if ~isempty(min_col) && ~isempty(min_row)
        % Si la pelota es detectada, actualizar con observación real
        zt = [min_col; min_row];
        K = P * H' / (H * P * H' + R);  % Ganancia de Kalman

        % Corrección en posición solo con las dos primeras filas de K
        xt(1:2) = xt(1:2) + K(1:2, :) * (zt - H * xt);

        % Suavizado de la posición estimada
        xt(1:2) = alpha * xt(1:2) + (1 - alpha) * zt;

        % Actualización de la covarianza solo en la parte superior izquierda
        KH_full = eye(4);
        KH_full(1:2, 1:4) = K(1:2, :) * H;
        P = (eye(4) - KH_full) * P;
    else
        % Si no se detecta, solo predecir sin corrección
        K = zeros(size(P * H'));
    end

    % Predicción del siguiente estado
    xt = A * xt;
    P = A * P * A' + Q;

    % Dibujar la posición estimada de la pelota
    im1 = insertShape(im1, 'Rectangle', [xt(1) xt(2) widthbbox highbbox], 'LineWidth', 2, 'Color', 'red');
    scenario_Kalman(:,:,:,k) = im1;
    
    % Ajuste adaptativo de ruido
    ax = (xt(3) - xtp(3)) / dt;
    ay = (xt(4) - xtp(4)) / dt;
    if abs(ax) >= 1 || abs(ay) >= 1
        Q = diag([0.5, 0.5, 2, 2]);
    else
        Q = diag([0.1, 0.1, 1, 1]);
    end
    
    % Actualización del estado previo
    xtp = xt;
end

% Reproducción y guardado del video con el tracking
implay(scenario_Kalman, frameRate);
v = VideoWriter('tracked_ball_output', 'MPEG-4');
v.FrameRate = frameRate;
open(v);
writeVideo(v, scenario_Kalman);
close(v);

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
