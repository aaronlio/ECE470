%% MyKuka Testing

%     theta d      a     alpha
DH = [0     400    25    pi/2;
      0     0      315   0;
      0     0      35    pi/2;
      0     365    0     -pi/2;
      0     0      0     pi/2;
      0     161.44 156  0];

myrobot = mykuka(DH);

H1 = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4],myrobot);
test_q = inverse_kuka(H1,myrobot);
plot(myrobot, test_q)

%           theta  d      a     alpha
DH_forces = [0     400    25    pi/2;
             0     0      315   0;
             0     0      35    pi/2;
             0     365    0     -pi/2;
             0     0      0     pi/2;
             0     161.44 0     0];

%% Lab4 Prep 4: test rep
setupobstacle_lab4prep;
tau = rep(myrobot,[pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],obs);




%% test att
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4)=100*[-1; 3; 3;]/4;
q1 = inverse_kuka(H1,myrobot);
H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4)=100*[3; -1; 2;]/4;
q2 = inverse_kuka(H2,myrobot);
tau = att(q1,q2,myrobot);

%% test motionplan without obs

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


%% test motionplan with obs
setupobstacle
hold on
axis([-100 100 -100 100 0 200])
view(-32,50)
plotobstacle(obs);
qref = motionplan(q1,q2,0,10,myrobot,obs,0.01);
t=linspace(0,10,300);
q=ppval(qref,t)';
plot(myrobot,q);
hold off