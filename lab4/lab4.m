%% MyKuka Testing
%     theta d      a     alpha
DH = [0     400    25    pi/2;
      0     0      315   0;
      0     0      35    pi/2;
      0     365    0     -pi/2;
      0     0      0     pi/2;
      0     161.44 -156  0];
  
%           theta  d      a     alpha
DH_forces = [0     400    25    pi/2;
             0     0      315   0;
             0     0      35    pi/2;
             0     365    0     -pi/2;
             0     0      0     pi/2;
             0     161.44 0     0];
  
kuka = mykuka(DH);
kuka_forces = mykuka(DH_forces);


%% Lab4 Prep 4: test rep
%plot(kuka_forces, [pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6])

setupobstacle_lab4prep;
tau = rep([pi/10,pi/12,pi/6,pi/2,pi/2,-pi/6],kuka_forces,prepobs{1});


%% Lab4 Prep 4: 
p1 = [620 375 50];
p2 = [620 -375 50];
R=[0 0 1;0 -1 0;1 0 0];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
t1 = 0;
t2 = 50;
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
qref = motionplan(q1, q2, t1, t2, kuka_forces, prepobs, 0.01);

%plot
hold on
t=linspace(0,10,300);
q=ppval(qref,t)';
view(-32,50)
axis([-1000 1000 -1000 1000 -1000 1000])
plotobstacle(prepobs);
plot(kuka,q);
hold off

%% 
setupobstacle;

q_home = [0 1.5708 0 0 1.5708 0];
H_home = forward_kuka(q_home, kuka);
%p_home = H_home(1:3, 4);
p0 = [370 -440 150];
p1 = [370 -440 45];
p2 = [750 -220 225];
p3 = [620 350 45];

R=[0 0 1;0 -1 0;1 0 0];

H0=[R p0';zeros(1,3) 1];
H1=[R p1';zeros(1,3) 1];
H2=[R p2';zeros(1,3) 1];
H3=[R p3';zeros(1,3) 1];

t1 = 0;
t2 = 50;
q0 = inverse_kuka(H0, kuka);
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
q3 = inverse_kuka(H3, kuka);

qref = motionplan(q_home, q0, t1, t2, kuka_forces, obs, 0.01);
qref2 = motionplan(q0, q1, t1, t2, kuka_forces, obs, 0.01);
qref3 = motionplan(q1, q2, t1, t2, kuka_forces, obs, 0.01);
qref4 = motionplan(q2, q3, t1, t2, kuka_forces, obs, 0.01);


%% plot
hold on
t=linspace(0,50,300);
q = ppval(qref,t)';
q1 = ppval(qref2,t)';
q2 = ppval(qref3,t)';
q3 = ppval(qref4,t)';

qT = [q; q1; q2; q3];
view(-32,50)
axis([-1000 1000 -1000 1000 -1000 1000])
plotobstacle(obs);
plot(kuka,qT);
hold off

%% 4.2

for i = 1:300
    setAngles(q(i,:), 0.03);
end

%%
setAngles(inverse_kuka(H1, kuka), 0.03);

%%
for i = 1:300
    setAngles(q2(i,:), 0.03);
end
for i = 1:300
    setAngles(q3(i,:), 0.03);
end