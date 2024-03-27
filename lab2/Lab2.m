%% MyKuka Testing

%     theta d      a     alpha
DH = [0     400    25    pi/2;
      0     0      315   0;
      0     0      35    pi/2;
      0     365    0     -pi/2;
      0     0      0     pi/2;
      0     161.44 -296.23  0];

kuka = mykuka(DH);

H1 = forward_kuka([pi/5 pi/3 -pi/4 pi/4 pi/3 pi/4],kuka);
test_q = inverse_kuka(H1,kuka);
plot(kuka, test_q)

