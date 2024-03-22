function tau = att(q, q2, myrobot)
F_att = zeros(3,6);
d = inf;
zeta = 1;

%initailize data structures
H = zeros(4,4,6);
H_final = zeros(4,4,6);
J_o = zeros(3,6,6);
z = zeros(3,7);
o = zeros(3,7);
o_final = zeros(3,7);
z(:,1) = [0;0;1];  %the first column of z stores z_0, and the second column stores z_1, etc
o(:,1) = [0;0;0];  %the first column of 0 stores 0_0, and the second column stores 0_1, etc
o_final(:,1) = [0;0;0];

%calculate o_i and z_i, i = 1 to 6
for i = 1:6
    H(:,:, i) = forward_kuka(q(1:i), myrobot);
    H_final(:,:,i) = forward_kuka(q2(1:i), myrobot);
    z(:,i+1) = H(1:3,3,i);
    o(:,i+1) = H(1:3,4,i);
    o_final(:,i+1) = H_final(1:3,4,i);
end

%Calculate F_att
for i = 1:6
    if norm(o(:,i+1)-o_final(:,i+1)) <= d
        F_att(:,i) = -zeta * (o(:,i+1)-o_final(:,i+1));
    else
        F_att(:,i) = -d * zeta * (o(:,i+1)-o_final(:,i+1))/(norm(o(:,i+1)-o_final(:,i+1)));
    end
end

F_att = F_att/100;

%Calculate Jacobian
for i = 1:6
    for j = 1:i
        J_o(:,j,i) = cross(z(:,j), o(:, i+1) - o(:,j)); 
    end
end

%Calculate tau
tau = [0; 0; 0; 0; 0; 0];
for i = 1:6
   tau = tau + J_o(:,:,i).' * F_att(:,i);
end

tau = tau.';
tau = tau/norm(tau);
end

