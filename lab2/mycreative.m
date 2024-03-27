%% MyKuka Testing

%     theta d      a     alpha
DH = [0     400    25    pi/2;
      0     0      315   0;
      0     0      35    pi/2;
      0     365    0     -pi/2;
      0     0      0     pi/2;
      0     161.44 -296.23  0];

kuka = mykuka(DH);

X_workspace = zeros(100,3)';
X_baseframe = zeros(100,3)';

for i = 1:25
 smallx = (1/25)*i;
 smally = (sqrt(1-smallx^2)+sqrt(smallx));
 X_workspace(:, i) = [620 + smallx*50 ; 0 + smally*50 ; -1];
end
for i = 1:25
 smallx = 1-(1/25)*i;
 smally = (-sqrt(1-smallx^2)+sqrt(smallx));
 X_workspace(:, i+25) = [620 + smallx*50 ; 0 + smally*50 ; -1];
end
for i = 1:25
 smallx = -(1/25)*i;
 smally = -(-sqrt(1-(smallx+1)^2)+sqrt(smallx+1));
 X_workspace(:, i+50) = [620 + smallx*50 ; 0 + smally*50 ; -1];
end
for i = 1:25
 smallx = -1 + (1/25)*i;
 smally = (sqrt(1-(smallx+1)^2)+sqrt(smallx+1));
 X_workspace(:, i+75) = [620 + smallx*50 ; 0 + smally*50 ; -1];
end

for i = 1:100
 X_baseframe(:, i) = FrameTransformation(X_workspace(:, i));
end

R0_6 = [0 0 1;
        0 -1 0;
        1 0 0];

H0_6 = [0 0 1 0; 
     0 -1 0 0; 
     1 0 0 0; 
     0 0 0 1];

q_res = zeros(100, 6);
pos_res = zeros(100, 3);

for i = 1:100
    H0_6(1:3, 4) = X_workspace(:, i);
    q_res(i, :) = inverse(H0_6, kuka);

    %command robot using setangles 

    % Validate forward and inverse methods by comparing given coordinates
    % to computed coordinates

    H_res = forward(q_res(i, :), kuka);
    pos_res(i, :) = H_res(1:3,4);
end

check = isequal(pos_res, o);

plot3(pos_res(:,1), pos_res(:,2), pos_res(:,3), 'b');
hold on
zlim([-220 150])
plot(kuka, q_res)