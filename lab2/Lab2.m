%% MyPuma Testing

DH = [0     76     0     pi/2;
      0     -23.65 43.23 0;
      0     0      0     pi/2;
      0     43.18  0     -pi/2;
      0     0      0     pi/2;
      0     20     0     0];
%     theta d      a     alpha
DH = [0     400    25    pi/2;
      0     0      315   0;
      0     0      35    pi/2;
      0     365    0     -pi/2;
      0     0      0     pi/2;
      0     161.44 -296.23  0];

myrobot = mykukua(DH);

ans1 = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4],kuka);
inverse_kuka(ans1,kuka);

%% Plot a sample joint space trajectory
theta1=linspace(0,pi,200);
theta2=linspace(0,pi/2,200);
theta3=linspace(0,pi,200);
theta4=linspace(pi/4,3*pi/4,200);
theta5=linspace(-pi/3,pi/3,200);
theta6=linspace(0,2*pi,200);

q = [theta1; theta2; theta3; theta4; theta5; theta6].';
%test = [0, 0, 0, 0, 0, 0];
test = [0.5, 0.4, 0.3, 0, pi/2, 0];
plot(myrobot, test)


%% Forward Test %%
H = zeros(4,4,200);
for i = 1:200
    H(:,:,i) = forward_kuka(q(i, :),myrobot);
end

o = squeeze(H(1:3, 4, :)).'; 

plot3(o(:,1),o(:,2),o(:,3),'r')
hold on

plot(myrobot, q)

%% Single Inverse Test

H = [cos(pi/4) -sin(pi/4) 0 20; sin(pi/4) cos(pi/4) 0 23; 0 0 1 15; 0 0 0 1];

q = inverse_kuka(H, myrobot);

%% Inverse Path Test
ox = linspace(10, 30, 100);
oy = linspace(10, 23, 100);
oz = linspace(15, 100, 100);

o = [ox oy oz];
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
    H(1:3, 4) = o(:, i).';
    q_res(i, :) = inverse_kuka(H, myrobot);

    % Validate forward and inverse methods by comparing given coordinates
    % to computed coordinates

    H_res = forward_kuka(q_res(i, :), myrobot);
    pos_res(i, :) = H_res(1:3,4);
end

check = isequal(pos_res, o);

plot3(o(:,1), o(:,2), o(:,3),'r');
hold on
plot(myrobot, q_res);
