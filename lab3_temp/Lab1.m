%% MyPuma Testing

% Write DH Table from prep
DH = [0     pi/2     76     0;
      43.23 0        -23.65 0;
      0     pi/2     0      0;
      0     -pi/2    43.18  0;
      0     pi/2     0      0;
      0     0        20     0];

% Create Robot
myrobot = mypuma560(DH);

%% Plot a sample joint space trajectory
theta1=linspace(0,pi,200).';
theta2=linspace(0,pi/2,200).';
theta3=linspace(0,pi,200).';
theta4=linspace(pi/4,3*pi/4,200).';
theta5=linspace(-pi/3,pi/3,200).';
theta6=linspace(0,2*pi,200).';

% Plot sample trajectory
q = [theta1 theta2 theta3 theta4 theta5 theta6];
plot(myrobot, q)

%% Forward Test %%
H = zeros(4,4,200);
for i = 1:200
    % compute joint variables from given homogenous transformation
    H(:,:,i) = forward(q(i, :),myrobot);
end

o = squeeze(H(1:3, 4, :)).'; 

% Plot trajectory
plot3(o(:,1),o(:,2),o(:,3),'r')
hold on
% Plot movement
plot(myrobot, q)

%% Single Inverse Test

H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];
% Compute single inverse
q = inverse(H, myrobot);

%% Inverse Path Test
ox = linspace(10, 30, 100);
oy = linspace(23, 30, 100);
oz = linspace(15, 100, 100);

o = [ox.' oy.' oz.'];
R = [cos(pi/4) -sin(pi/4) 0;
        sin(pi/4), cos(pi/4) 0;
        0 0 1];

H = [cos(pi/4) -sin(pi/4) 0 20; 
    sin(pi/4) cos(pi/4) 0 23; 
    0 0 1 15; 
    0 0 0 1];

q_res = zeros(100, 6);
pos_res = zeros(100, 3);

for i = 1:100
    H(1:3, 4) = o(i, :).';
    q_res(i, :) = inverse(H, myrobot);

    % Validate forward and inverse methods by comparing given coordinates
    % to computed coordinates

    H_res = forward(q_res(i, :), myrobot);
    pos_res(i, :) = H_res(1:3,4);
end

check = isequal(pos_res, o);

plot3(pos_res(:,1), pos_res(:,2), pos_res(:,3), 'b');
hold on
zlim([-220 150])
plot(myrobot, q_res)

