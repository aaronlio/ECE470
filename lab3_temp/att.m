function tau = att(q, q2, myrobot)

%q2(4) = q2(4) + 2*pi;

F_att = zeros(3,6);

d = inf;
zeta = 1;

H = zeros(4,4,6);
H_final = zeros(4,4,6);
J_o = zeros(3,6,6);
z = zeros(3,7);
o = zeros(3,7);
o_final = zeros(3,7);
z(:,1) = [0;0;1];
o(:,1) = [0;0;0];
o_final(:,1) = [0;0;0];



for i = 1:6
    H(:,:, i) = forward(q(1:i), myrobot);
    H_final(:,:,i) = forward(q2(1:i), myrobot);
    z(:,i+1) = H(1:3,3,i);
    o(:,i+1) = H(1:3,4,i);
    o_final(:,i+1) = H_final(1:3,4,i);
end



for i = 1:6
    if norm(o(:,i+1)-o_final(:,i+1)) <= d
        F_att(:,i) = -zeta * (o(:,i+1)-o_final(:,i+1));
    else
        F_att(:,i) = -d * zeta * (o(:,i+1)-o_final(:,i+1))/(norm(o(:,i+1)-o_final(:,i+1)));
    end
end

F_att = F_att/100;




for i = 1:6
    for j = 1:i
        J_o(:,j,i) = cross(z(:,j), o(:, i+1) - o(:,j)); 
    end
end

tau = [0; 0; 0; 0; 0; 0];
for i = 1:6
   tau = tau + J_o(:,:,i).' * F_att(:,i);
end

tau = tau.';
tau = tau/norm(tau);
end

