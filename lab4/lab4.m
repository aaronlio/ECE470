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
t2 = 10;
q1 = inverse_kuka(H1, kuka);
q2 = inverse_kuka(H2, kuka);
qref = motionplan(q1, q2, t1, t2, kuka_forces, prepobs, 0.03);

%plot
hold on
t=linspace(0,10,300);
q=ppval(qref,t)';
view(-32,50)
axis([-1000 1000 -1000 1000 -1000 1000])
plotobstacle(prepobs);
plot(kuka,q);
hold off

