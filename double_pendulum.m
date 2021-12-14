%% Double pendulum
% Simulation and animation of a double pendulum.
%
%%

clear ; close all ; clc

%% Parameters
mA  = 1;                        % Mass A                [kg]
mB  = 0.1;                        % Mass B                [kg]
rA  = 5;                        % Length of rod A       [m]
rB  = 5;                        % Length of rod B       [m]
g   = 9.81;                     % Gravity               [m/s2]

parameters = [mA mB rA rB g];

%% Initial conditions
thA0    = pi;            % Orientation rod A     [rad]
thB0    = pi;            % Orientation rod B     [rad]
dthA0   = 2;                    % Angular speed A       [rad/s]
dthB0   = -2;                    % Angular speed B       [rad/s]

x0 = [thA0 thB0 dthA0 dthB0];

%% Simulation
tf      = 60;                   % Final time            [s]
fR      = 60;                   % Frame rate            [fps]
dt      = 1/fR;                 % Time resolution       [s]
time    = linspace(0,tf,tf*fR); % Time                  [s]

options = odeset('RelTol',1e-6);
[TOUT,XOUT] = ode45(@(t,x) double_pendulum_model(t,x,parameters),time,x0);

% Retrieving states
thA     = XOUT(:,1);
thB     = XOUT(:,2);
% dthA    = XOUT(:,3);
% dthB    = XOUT(:,4);

% Position A
mAPosX = rA*sin(thA);
mAPosY = -rA*cos(thA);

% Position B/A
mBAPosX = rB*sin(thB);
mBAPosY = -rB*cos(thB);

% Position B
mBPosX = mBAPosX + mAPosX;
mBPosY = mBAPosY + mAPosY;

%% Animation

c = cool(6); % Colormap

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

hold on ; grid on ; box on ; axis equal
set(gca,'XLim',[-1.1*(rA+rB) 1.1*(rA+rB)])
set(gca,'YLim',[-1.1*(rA+rB) 1.1*(rA+rB)])
set(gca,'XTick',[],'YTick',[])
set(gca,'Color','k')

% Create and open video writer object
v = VideoWriter('double_pendulum.mp4','MPEG-4');
v.Quality   = 100;
v.FrameRate = fR;
open(v);

for i = 1:length(mAPosX)
    cla

    % Rod 1
    plot([0 mAPosX(i)],[0 mAPosY(i)],'w','LineWidth',2)
    % Rod 2
    plot([mAPosX(i) mBPosX(i)],[mAPosY(i) mBPosY(i)],'w','LineWidth',2)

    % Origin
    p = plot(0,0,'wo');
    set(p,'MarkerFaceColor','w','MarkerSize',5)

    % Trajectory A
    plot(mAPosX(1:i),mAPosY(1:i),'color',c(6,:),'LineWidth',3)
    % Trajecotry B
    plot(mBPosX(1:i),mBPosY(1:i),'color',c(4,:),'LineWidth',3)
    
    % Position A
    p = plot(mAPosX(i),mAPosY(i),'r');
    set(p,'Marker','o','MarkerFaceColor',c(6,:),'Color','k','MarkerSize',15)
    % Position B
    p = plot(mBPosX(i),mBPosY(i),'g');
    set(p,'Marker','o','MarkerFaceColor',c(4,:),'Color','k','MarkerSize',15)

    frame = getframe(gcf);
    writeVideo(v,frame);
end

close(v);

%% Auxiliary function

function dx = double_pendulum_model(~,x,parameters)
    % Parameters
    mA  = parameters(1);        % Mass A                [kg]
    mB  = parameters(2);        % Mass B                [kg]
    rA  = parameters(3);        % Length rod A          [m]
    rB  = parameters(4);        % Length rod B          [m]
    g   = parameters(5);        % Gravity               [m/s2]

    % States
    thA     = x(1);
    thB     = x(2);
    dthA    = x(3);
    dthB    = x(4);

    % State equations
    ddthA   = -(mB*rA*cos(thA - thB)*sin(thA - thB)*dthA^2 + mB*rB*sin(thA - thB)*dthB^2 + g*mA*sin(thA) + g*mB*sin(thA) - g*mB*cos(thA - thB)*sin(thB))/(rA*(- mB*cos(thA - thB)^2 + mA + mB));
    ddthB   = (g*mA*sin(2*thA - thB) - g*mB*sin(thB) - g*mA*sin(thB) + g*mB*sin(2*thA - thB) + 2*dthA^2*mA*rA*sin(thA - thB) + 2*dthA^2*mB*rA*sin(thA - thB) + dthB^2*mB*rB*sin(2*thA - 2*thB))/(rB*(2*mA + mB - mB*cos(2*thA - 2*thB)));
    
    dx = [dthA ; dthB ; ddthA ; ddthB];
end
