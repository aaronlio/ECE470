% Write DH Table from prep
DH = [0     pi/2     76     0;
      43.23 0        -23.65 0;
      0     pi/2     0      0;
      0     -pi/2    43.18  0;
      0     pi/2     0      0;
      0     0        20     0];

% Create Robot
myrobot = mypuma560(DH);

%% test att
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse(H2,myrobot);
tau = att(q1,q2,myrobot);

%% test motionplan

qref = motionplan(q1,q2, 0, 10, myrobot,[],0.01);
t = linspace(0,10,300);
q = ppval(qref, t)';
plot(myrobot,q)

%% test repulsive
setupobstacle
q3 = 0.9 * q1 + 0.1*q2;
tau_1 = rep(q3, myrobot, obs{1});

q_temp = [pi/2 pi 1.2*pi 0 0 0];
tau_temp = rep(q_temp, myrobot, obs{6});