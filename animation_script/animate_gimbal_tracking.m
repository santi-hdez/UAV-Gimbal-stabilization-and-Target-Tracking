%% UAV Gimbal Stabilization and Moving-Target Tracking

%% ========================================================================
% READ SIMULINK DATA
% =========================================================================

% -------------------------------------------------------------------------
% Kinematics data
% -------------------------------------------------------------------------

t = out.kinematics_data.Time(:);

K = squeeze(out.kinematics_data.Data);

% Ensure matrix format is [time samples x signals]

if size(K,1) ~= length(t) && size(K,2) == length(t)

    K = K.';

end


% -------------------------------------------------------------------------
% Tracking data
% -------------------------------------------------------------------------

tTracking = out.tracking_data.Time(:);

T0 = squeeze(out.tracking_data.Data);

if size(T0,1) ~= length(tTracking) && size(T0,2) == length(tTracking)

    T0 = T0.';

end


%% ========================================================================
% SYNCHRONIZE DATA
% =========================================================================

tStart = max(t(1),tTracking(1));
tEnd   = min(t(end),tTracking(end));

valid = (t >= tStart) & (t <= tEnd);

t = t(valid);
K = K(valid,:);


% Interpolate tracking results onto the kinematics time vector.

T = interp1(tTracking, T0, t, 'linear');


%% ========================================================================
% EXTRACT SIGNALS
% =========================================================================

% Target position [m]
xT = K(:,1);
yT = K(:,2);
zT = K(:,3);


% UAV position [m]
xU = K(:,4);
yU = K(:,5);
zU = K(:,6);


% UAV attitude [rad]
roll  = K(:,7);
pitch = K(:,8);
yaw   = K(:,9);


% Gimbal/controller data
tau_az = T(:,1);
tau_el = T(:,2);

az = T(:,3);
el = T(:,4);


%% ========================================================================
% VISUALIZATION PARAMETERS
% =========================================================================

% Draw every Nth simulation point
skip = 5;

% Duration of trajectory history shown [s]
trailDuration = 40;


% UAV visual scale.
%
% This is intentionally much larger than the physical aircraft would be
% relative to the simulated distances.
uavScale = 100;


% Length of displayed UAV body axes
axisLength = 170;


% Camera field-of-view HALF angle
fovHalfAngle = deg2rad(4);


% Number of points used to construct the FOV cone
nCone = 50;


% Smoothing coefficient for the following visualization camera
cameraAlpha = 0.9;


% Initial centre of visualization
cameraCenter = [xU(1); yU(1); zU(1)];


%% ========================================================================
% CREATE FIGURE
% =========================================================================

fig = figure( ...
    'Name','UAV Gimbal Stabilization and Target Tracking', ...
    'Position',[40 40 1700 950], ...
    'Color',[0.07 0.07 0.07]);


ax = axes(fig, 'Position',[0.055 0.09 0.70 0.84]);


hold(ax,'on');
grid(ax,'on');
box(ax,'on');


% Dark background
ax.Color = [0.04 0.05 0.055];


% Axis colors
ax.XColor = [0.90 0.90 0.90];
ax.YColor = [0.90 0.90 0.90];
ax.ZColor = [0.90 0.90 0.90];


% Grid appearance
ax.GridColor = [0.65 0.65 0.65];
ax.GridAlpha = 0.18;


ax.FontSize = 13;
ax.LineWidth = 1;


xlabel(ax,'X [m]', 'FontSize',15, 'FontWeight','bold');

ylabel(ax,'Y [m]', 'FontSize',15, 'FontWeight','bold');

zlabel(ax,'Z [m]', 'FontSize',15, 'FontWeight','bold');


title(ax, ...
    'UAV Gimbal Stabilization & Target Tracking', ...
    'FontSize',17, ...
    'FontWeight','bold', ...
    'Color',[0.95 0.95 0.95]);


% Visualization viewing direction
view(ax,45,24);

axis(ax,'vis3d');


%% ========================================================================
% GROUND PLANE
% =========================================================================

groundSize = 5000;


[Xg,Yg] = meshgrid([-groundSize groundSize], [-groundSize groundSize]);


Zg = zeros(size(Xg));


surf(ax,Xg,Yg,Zg, ...
    'FaceColor',[0.08 0.16 0.16], ...
    'FaceAlpha',0.30, ...
    'EdgeColor','none');


%% ========================================================================
% UAV GEOMETRY IN BODY COORDINATES
% =========================================================================
%
% Coordinate convention:
%
%       +z_B
%         |
%         |
%         +-------- +x_B   forward
%        /
%      +y_B
%
% Zero gimbal azimuth/elevation therefore corresponds to the camera
% pointing along +x_B.

% Note: UAV and target are not correctly physically scaled for
% visualization purposes.

% -------------------------------------------------------------------------
% Fuselage
% -------------------------------------------------------------------------

fuselage_B = uavScale * [ ...

     1.00   0.00   0.00;      % nose

     0.25   0.18   0.08;

    -0.75   0.14   0.06;

    -0.95   0.00   0.00;      % rear

    -0.75  -0.14   0.06;

     0.25  -0.18   0.08

]';


% -------------------------------------------------------------------------
% Main wing
% -------------------------------------------------------------------------

wing_B = uavScale * [ ...

     0.20   0.00   0.00;

    -0.10   1.00   0.00;

    -0.35   0.95   0.00;

    -0.15   0.00   0.00;

    -0.35  -0.95   0.00;

    -0.10  -1.00   0.00

]';


% -------------------------------------------------------------------------
% Horizontal tail
% -------------------------------------------------------------------------

tail_B = uavScale * [ ...

    -0.65   0.00   0.02;

    -0.88   0.45   0.02;

    -0.95   0.00   0.02;

    -0.88  -0.45   0.02

]';


% -------------------------------------------------------------------------
% Vertical stabilizer
% -------------------------------------------------------------------------

fin_B = uavScale * [ ...

    -0.65   0.00   0.00;

    -0.90   0.00   0.00;

    -0.82   0.00   0.40

]';


% -------------------------------------------------------------------------
% Visual gimbal mounting position
% -------------------------------------------------------------------------

gimbalOffset_B = uavScale * [0.12; 0.00; -0.18];


%% ========================================================================
% CREATE UAV GRAPHICS
% =========================================================================

hFuselage = patch(ax, ...
    'XData',nan, ...
    'YData',nan, ...
    'ZData',nan, ...
    'FaceColor',[0.20 0.60 0.90], ...
    'FaceAlpha',0.95, ...
    'EdgeColor',[0.65 0.85 1.00], ...
    'LineWidth',1.3);


hWing = patch(ax, ...
    'XData',nan, ...
    'YData',nan, ...
    'ZData',nan, ...
    'FaceColor',[0.16 0.48 0.75], ...
    'FaceAlpha',0.95, ...
    'EdgeColor',[0.65 0.85 1.00], ...
    'LineWidth',1.2);


hTail = patch(ax, ...
    'XData',nan, ...
    'YData',nan, ...
    'ZData',nan, ...
    'FaceColor',[0.16 0.48 0.75], ...
    'FaceAlpha',0.95, ...
    'EdgeColor',[0.65 0.85 1.00], ...
    'LineWidth',1.2);


hFin = patch(ax, ...
    'XData',nan, ...
    'YData',nan, ...
    'ZData',nan, ...
    'FaceColor',[0.20 0.60 0.90], ...
    'FaceAlpha',0.95, ...
    'EdgeColor',[0.65 0.85 1.00], ...
    'LineWidth',1.2);


%% ========================================================================
% GIMBAL GRAPHICS
% =========================================================================

% Create unit sphere
[sx,sy,sz] = sphere(18);


% Visual gimbal radius
gimbalRadius = 0.15*uavScale;


hGimbal = surf(ax, ...
    nan(size(sx)), ...
    nan(size(sy)), ...
    nan(size(sz)), ...
    'FaceColor',[0.95 0.55 0.15], ...
    'EdgeColor','none', ...
    'FaceAlpha',1);


%% ========================================================================
% TARGET GRAPHICS
% =========================================================================

hTarget = plot3(ax,nan,nan,nan, ...
    'o', ...
    'MarkerSize',25, ...
    'MarkerEdgeColor',[1.00 0.85 0.10], ...
    'MarkerFaceColor',[1.00 0.65 0.05], ...
    'LineWidth',2.5);


hTargetStem = plot3(ax,nan,nan,nan, ...
    '-', ...
    'Color',[1.00 0.65 0.05], ...
    'LineWidth',3);


%% ========================================================================
% TRAJECTORIES
% =========================================================================

hUAVtrail = plot3(ax,nan,nan,nan, ...
    '--', ...
    'Color',[0.60 0.35 1.00], ...
    'LineWidth',1.8);


hTargetTrail = plot3(ax,nan,nan,nan, ...
    '--', ...
    'Color',[0.25 0.90 0.35], ...
    'LineWidth',1.8);


%% ========================================================================
% LOS AND CAMERA BORESIGHT
% =========================================================================

% True geometrical LOS
hLOS = plot3(ax,nan,nan,nan, ...
    '--', ...
    'Color',[0.15 0.90 1.00], ...
    'LineWidth',2);


% Actual camera optical axis
hCamera = plot3(ax,nan,nan,nan, ...
    '-', ...
    'Color',[1.00 0.20 0.60], ...
    'LineWidth',2.5);


%% ========================================================================
% FIELD-OF-VIEW CONE
% =========================================================================

hFOV = surf(ax, ...
    nan(2,nCone+1), ...
    nan(2,nCone+1), ...
    nan(2,nCone+1), ...
    'FaceColor',[1.00 0.25 0.60], ...
    'FaceAlpha',0.2, ...
    'EdgeColor','none');


hFOVring = plot3(ax,nan,nan,nan, ...
    '-', ...
    'Color',[1.00 0.35 0.65], ...
    'LineWidth',1.3);


%% ========================================================================
% UAV BODY AXES
% =========================================================================

% x_B = red
hXB = quiver3(ax,0,0,0,0,0,0, ...
    'Color',[1.00 0.25 0.25], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.6);


% y_B = green
hYB = quiver3(ax,0,0,0,0,0,0, ...
    'Color',[0.25 1.00 0.25], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.6);


% z_B = blue
hZB = quiver3(ax,0,0,0,0,0,0, ...
    'Color',[0.25 0.55 1.00], ...
    'LineWidth',2, ...
    'MaxHeadSize',0.6);


%% ========================================================================
% LEGEND
% =========================================================================

% Graphical object used only for the UAV legend symbol
hUAVlegend = plot3(ax,nan,nan,nan, ...
    '-', ...
    'Color',[0.20 0.60 0.90], ...
    'LineWidth',5);


lgd = legend(ax, ...
    [ ...
     hUAVlegend, ...
     hTarget, ...
     hUAVtrail, ...
     hTargetTrail, ...
     hLOS, ...
     hCamera ...
    ], ...
    { ...
     'UAV', ...
     'Target', ...
     'UAV trajectory', ...
     'Target trajectory', ...
     'True LOS', ...
     'Camera boresight' ...
    });


lgd.Position = [0.79 0.43 0.19 0.25];

lgd.FontSize = 12;

lgd.TextColor = [0.95 0.95 0.95];

lgd.Color = [0.10 0.10 0.10];

lgd.EdgeColor = [0.35 0.35 0.35];


%% ========================================================================
% TELEMETRY PANEL
% =========================================================================

hInfo = annotation(fig,'textbox', ...
    [0.79 0.68 0.19 0.25], ...
    'FitBoxToText','off', ...
    'FontName','Monospaced', ...
    'FontSize',12, ...
    'FontWeight','bold', ...
    'Color',[0.95 0.95 0.95], ...
    'BackgroundColor',[0.10 0.10 0.10], ...
    'EdgeColor',[0.35 0.35 0.35], ...
    'LineWidth',1.2, ...
    'Margin',12);

%% ========================================================================
% ANIMATION LOOP
% =========================================================================

for k = 1:skip:length(t)


    %% --------------------------------------------------------------------
    % UAV AND TARGET POSITIONS
    % ---------------------------------------------------------------------

    pU = [xU(k); yU(k); zU(k)];


    pT = [xT(k); yT(k); zT(k)];


    %% --------------------------------------------------------------------
    % UAV ATTITUDE
    % ---------------------------------------------------------------------

    phi   = roll(k);
    theta = pitch(k);
    psi   = yaw(k);


    % Roll
    Rx = [ ...
        1, 0,         0;
        0, cos(phi), -sin(phi);
        0, sin(phi),  cos(phi)];


    % Pitch
    Ry = [ ...
         cos(theta), 0, sin(theta);
         0,          1, 0;
        -sin(theta), 0, cos(theta)];


    % Yaw
    Rz = [ ...
        cos(psi), -sin(psi), 0;
        sin(psi),  cos(psi), 0;
        0,         0,        1];


    % Body -> World rotation matrix
    R = Rz*Ry*Rx;


    %% --------------------------------------------------------------------
    % TRANSFORM UAV GEOMETRY BODY -> WORLD
    % ---------------------------------------------------------------------

    fuselage_W = R*fuselage_B + pU;

    wing_W = R*wing_B + pU;

    tail_W = R*tail_B + pU;

    fin_W = R*fin_B + pU;


    set(hFuselage, ...
        'XData',fuselage_W(1,:), ...
        'YData',fuselage_W(2,:), ...
        'ZData',fuselage_W(3,:));


    set(hWing, ...
        'XData',wing_W(1,:), ...
        'YData',wing_W(2,:), ...
        'ZData',wing_W(3,:));


    set(hTail, ...
        'XData',tail_W(1,:), ...
        'YData',tail_W(2,:), ...
        'ZData',tail_W(3,:));


    set(hFin, ...
        'XData',fin_W(1,:), ...
        'YData',fin_W(2,:), ...
        'ZData',fin_W(3,:));


    %% --------------------------------------------------------------------
    % VISUAL GIMBAL POSITION
    % --------------------------------------------------------------------

    pGimbal = pU + R*gimbalOffset_B;

    set(hGimbal, ...
        'XData',pGimbal(1) + gimbalRadius*sx, ...
        'YData',pGimbal(2) + gimbalRadius*sy, ...
        'ZData',pGimbal(3) + gimbalRadius*sz);


    %% --------------------------------------------------------------------
    % TARGET
    % ---------------------------------------------------------------------

    set(hTarget, ...
        'XData',pT(1), ...
        'YData',pT(2), ...
        'ZData',pT(3)+5);


    set(hTargetStem, ...
        'XData',[pT(1),pT(1)], ...
        'YData',[pT(2),pT(2)], ...
        'ZData',[pT(3),pT(3)+20]);


    %% --------------------------------------------------------------------
    % RECENT TRAJECTORIES
    % ---------------------------------------------------------------------

    trailStart = find( ...
        t >= t(k)-trailDuration, ...
        1, ...
        'first');


    set(hUAVtrail, ...
        'XData',xU(trailStart:k), ...
        'YData',yU(trailStart:k), ...
        'ZData',zU(trailStart:k));


    set(hTargetTrail, ...
        'XData',xT(trailStart:k), ...
        'YData',yT(trailStart:k), ...
        'ZData',zT(trailStart:k));


    %% --------------------------------------------------------------------
    % TRUE LOS
    % --------------------------------------------------------------------

    rLOS = pT-pU;

    distance = norm(rLOS);

    if distance > 0

        uLOS = rLOS/distance;

    else

        uLOS = [1;0;0];

    end


    % For visualization, draw the LOS starting at the graphical
    % gimbal position while retaining the correct LOS direction.

    pLOSEnd = pGimbal + distance*uLOS;


    set(hLOS, ...
        'XData',[pGimbal(1),pLOSEnd(1)], ...
        'YData',[pGimbal(2),pLOSEnd(2)], ...
        'ZData',[pGimbal(3),pLOSEnd(3)]);


    %% --------------------------------------------------------------------
    % ACTUAL CAMERA DIRECTION
    % --------------------------------------------------------------------
    %
    % The actual azimuth and elevation define the camera direction in
    % UAV BODY coordinates.
    %
    %                  [ cos(el) cos(az) ]
    %       u_cam,B =  [ cos(el) sin(az) ]
    %                  [     sin(el)      ]
    %

    uCamB = [ ...

        cos(el(k))*cos(az(k));

        cos(el(k))*sin(az(k));

        sin(el(k))];


    % Transform camera direction:
    % Body -> World

    uCamW = R*uCamB;


    % Numerical normalization
    uCamW = uCamW/norm(uCamW);


    % Camera boresight endpoint
    pCamera = pGimbal + distance*uCamW;


    set(hCamera, ...
        'XData',[pGimbal(1),pCamera(1)], ...
        'YData',[pGimbal(2),pCamera(2)], ...
        'ZData',[pGimbal(3),pCamera(3)]);


    %% --------------------------------------------------------------------
    % FIELD-OF-VIEW CONE
    % --------------------------------------------------------------------
    %
    % Construct two unit vectors perpendicular to the camera boresight.
    %
    % Together:
    %
    %       uCamW, e1, e2
    %
    % form an orthogonal coordinate system around the camera axis.

    if abs(dot(uCamW,[0;0;1])) < 0.9

        reference = [0;0;1];

    else

        reference = [0;1;0];

    end


    e1 = cross(uCamW,reference);

    e1 = e1/norm(e1);


    e2 = cross(uCamW,e1);

    e2 = e2/norm(e2);


    % Cone length
    coneLength = distance-40;


    % Radius corresponding to the chosen FOV half-angle
    coneRadius = coneLength*tan(fovHalfAngle);


    % Angular coordinate around the cone
    beta = linspace(0,2*pi,nCone+1);


    % Circle defining the end of the cone
    coneCircle = pGimbal + coneLength*uCamW + coneRadius*(e1*cos(beta) ...
            + e2*sin(beta));


    % Cone surface
    coneX = [ ...
        repmat(pGimbal(1),1,nCone+1);
        coneCircle(1,:)];


    coneY = [ ...
        repmat(pGimbal(2),1,nCone+1);
        coneCircle(2,:)];


    coneZ = [ ...
        repmat(pGimbal(3),1,nCone+1);
        coneCircle(3,:)];


    set(hFOV, ...
        'XData',coneX, ...
        'YData',coneY, ...
        'ZData',coneZ);


    set(hFOVring, ...
        'XData',coneCircle(1,:), ...
        'YData',coneCircle(2,:), ...
        'ZData',coneCircle(3,:));


    %% --------------------------------------------------------------------
    % UAV BODY COORDINATE SYSTEM
    % --------------------------------------------------------------------


    xb = R(:,1);

    yb = R(:,2);

    zb = R(:,3);


    set(hXB, ...
        'XData',pU(1), ...
        'YData',pU(2), ...
        'ZData',pU(3), ...
        'UData',axisLength*xb(1), ...
        'VData',axisLength*xb(2), ...
        'WData',axisLength*xb(3));


    set(hYB, ...
        'XData',pU(1), ...
        'YData',pU(2), ...
        'ZData',pU(3), ...
        'UData',axisLength*yb(1), ...
        'VData',axisLength*yb(2), ...
        'WData',axisLength*yb(3));


    set(hZB, ...
        'XData',pU(1), ...
        'YData',pU(2), ...
        'ZData',pU(3), ...
        'UData',axisLength*zb(1), ...
        'VData',axisLength*zb(2), ...
        'WData',axisLength*zb(3));


    %% --------------------------------------------------------------------
    % TRUE 3D POINTING ERROR
    % --------------------------------------------------------------------
    %
    % Both uLOS and uCamW are expressed in WORLD coordinates.
    %


    cosError = dot(uLOS,uCamW);


    % Numerical protection for acos()
    cosError = max(-1,min(1,cosError));


    pointingError = acos(cosError);


    %% --------------------------------------------------------------------
    % SMOOTH FOLLOWING VIEW
    % --------------------------------------------------------------------

    desiredCenter = 0.5*(pU+pT);


    cameraCenter = ...
        (1-cameraAlpha)*cameraCenter ...
        + cameraAlpha*desiredCenter;


    % Determine viewing window from UAV-target separation
    viewRange = max(1.4*distance,800);


    xlim(ax,[ ...
        cameraCenter(1)-0.80*viewRange, ...
        cameraCenter(1)+0.80*viewRange]);


    ylim(ax,[ ...
        cameraCenter(2)-0.80*viewRange, ...
        cameraCenter(2)+0.80*viewRange]);


    % Keep ground and UAV visible
    zUpper = max([ ...
        600, ...
        pU(3)+150, ...
        pT(3)+150]);


    zlim(ax,[0,zUpper]);


    %% --------------------------------------------------------------------
    % TELEMETRY PANEL
    % --------------------------------------------------------------------

    infoText = sprintf( ...
        ['UAV GIMBAL TRACKING\n\n' ...
        'Time:           %7.2f s\n' ...
        'Azimuth:        %7.2f deg\n' ...
        'Elevation:      %7.2f deg\n\n' ...
        '3D point error: %7.3f deg\n\n' ...
        'Az torque:      %7.3f N m\n' ...
        'El torque:      %7.3f N m'], ...
        t(k), ...
        rad2deg(az(k)), ...
        rad2deg(el(k)), ...
        rad2deg(pointingError), ...
        tau_az(k), ...
        tau_el(k));

    hInfo.String = infoText;


    %% --------------------------------------------------------------------
    % UPDATE TITLE
    % --------------------------------------------------------------------

    titleText = sprintf( ...
        'UAV Gimbal Stabilization & Target Tracking   |   t = %.1f s', ...
        t(k));

    title(ax, titleText, ...
        'FontSize', 17, ...
        'FontWeight', 'bold', ...
        'Color', [0.95 0.95 0.95]);


    %% --------------------------------------------------------------------
    % RENDER FRAME
    % --------------------------------------------------------------------

    drawnow limitrate


end