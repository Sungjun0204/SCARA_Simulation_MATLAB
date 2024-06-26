% function myLoopingFcn()

%% Robot limits

THETA1_MAX = deg2rad(200);
THETA1_MIN = deg2rad(0);
THETA2_MAX = deg2rad(135);
THETA2_MIN = deg2rad(0);
D3_MAX = 0.200; % meters
D3_MIN = 0.00;
THETA4_MAX = deg2rad(180);
THETA4_MIN = deg2rad(-180);


%% Work envelope limits
MIN_SCARA = 0.28;
MAX_SCARA = 0.65;
MIN_USER = 0.25;
MAX_USER = 0.50;

%% DH Parameters
a1 = 0.400;   % 링크 1번 
a2 = 0.250;   % 링크 2번
d1 = 0.250;   % 링크 3번 (prismatic joint)
d4 = 0.050;   % 링크 3번 끝부분에서부터 end effect까지 거리

%% Links Definition
% [theta d a alpha R/P]
% R/P = 0: revolute joint
% R/P = 1: prismatic joint

L0 = Link([0, d1, 0, 0, 0], 'standard');

L1 = Link([0, 0, a1, pi, 0], 'standard');   % variable = theta1
L2 = Link([0, 0, a2, 0, 0], 'standard');    % variable = theta2
L3 = Link([0, 0, 0, 0, 1], 'standard');      % variable = d3 
L4 = Link([0, d4, 0, 0, 0], 'standard');    % variable = theta4


%%
global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;
gcf
set(gcf, 'KeyPressFcn', @myKeyPressFcn)

%% Definition to Robot

% Scara Robot Definition
scara = SerialLink([L0, L1, L2, L3, L4], 'name', 'SCARA');

% Figure
hold on;
grid on;
view([-37.5 30])    % isometric view
axis([-1 1 -1 1 0 1]);   % range of axis
xlabel('x');
ylabel('y');
zlabel('z');

time = 0;

while ~KEY_IS_PRESSED
    
    % 경로 생성 
    x = deg2rad(time);
    time = time + 1;

    px = 0.4*sin(x);
    py = 0.4;
    pz = abs(0.2*sin(x));


    % inverse kinematics
    theta2 = acos((px^2+py^2-a1^2-a2^2) / (2*a1*a2));
    beta = atan(py/px);
    if beta < 0
        beta = beta + pi;
    end
    phi = acos((px^2+py^2+a1^2-a2^2) / (2*sqrt(px^2+py^2)*a1));
    theta1 = beta + phi;

    if theta1 > THETA1_MAX
        theta1 = THETA1_MAX;
    elseif theta2 < THETA2_MIN
        theta2 = THETA2_MIN;
    end

    d3 = pz;
    if d3 > D3_MAX
        d3 = D3_MAX;
    elseif d3 < D3_MIN
        d3 = D3_MIN;
    end
    
    % plot setting
    plot(scara, [0 theta1 theta2 d3 0], 'workspace', axis);
%     plot3(px,py, (0.3-pz-d4), '.', 'LineWidth', 2, 'Color', 'red');

    pause(0.01);
    
end

disp('loop ended')

%%
function myKeyPressFcn(hObject, event)
global KEY_IS_PRESSED
KEY_IS_PRESSED = 1;
disp('key_is_pressed')
end

















