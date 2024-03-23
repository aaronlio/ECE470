function tau = rep(q, myrobot, obs)

F_rep = zeros(3,6);
zeta = 1;


H = zeros(4,4,6);
J_o = zeros(3,6,6);
z = zeros(3,7);
o = zeros(3,7);

z(:,1) = [0;0;1];
o(:,1) = [0;0;0];


for i = 1:6
    H(:,:, i) = forward(q(1:i), myrobot);
    z(:,i+1) = H(1:3,3,i);
    o(:,i+1) = H(1:3,4,i);
end

rho = zeros(1,6);
if strcmp(obs.type, 'cyl')
    R = obs.R;
    c = obs.c;
    rho0 = obs.rho0;
    h = obs.h;
    for i = 1:6
        rho(i) = sqrt((o(1,i+1) - c(1))^2 + (o(2,i+1) - c(2))^2) - R;
        rhoi_v = ((o(:, i+1) - [c(1); c(2); o(3, i+1)])/norm(o(:, i+1) - [c(1); c(2); o(3, i+1)])) * rho(i);
        if rho(i) <= rho0
            F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v));
        else
            F_rep(:, i) = [0;0;0];
        end
    end
    
elseif strcmp(obs.type, 'sph')
    R = obs.R;
    c = obs.c;
    rho0 = obs.rho0;
    
    for i = 1:6
        rho(i) = norm(o(:,i+1) - c) - R;
        rhoi_v = (o(:, i+1) - c)/norm(o(:, i+1) - c) * rho(i);
        if rho(i) <= rho0
            F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v));
        else
            F_rep(:, i) = [0;0;0];
        end
    end
end

F_rep = F_rep * 1000000;



for i = 1:6
    for j = 1:i
        J_o(:,j,i) = cross(z(:,j), o(:, i+1) - o(:,j)); 
    end
end

tau = [0; 0; 0; 0; 0; 0];
for i = 1:6
   tau = tau + J_o(:,:,i).' * F_rep(:,i);
end

tau = tau.';
tau = tau/norm(tau); 

end