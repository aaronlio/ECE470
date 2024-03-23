function tau = rep(q,myrobot,obs)
%initailize data structures
F_rep = zeros(3,6);
zeta = 1;
H = zeros(4,4,6);
J_o = zeros(3,6,6);
z = zeros(3,7);%the first column of z stores z_0, and the second column stores z_1, etc
o = zeros(3,7);%the first column of 0 stores 0_0, and the second column stores 0_1, etc

z(:,1) = [0;0;1];
o(:,1) = [0;0;0];

%calculate o_i and z_i, i = 1 to 6
for i = 1:6
    H(:,:, i) = forward_kuka(q(1:i), myrobot);
    z(:,i+1) = H(1:3,3,i);
    o(:,i+1) = H(1:3,4,i);
end

%calculate F_rep for cylinder or sphere
rho = zeros(1,6);
%cylinder
if strcmp(obs.type, 'cyl')
    R = obs.R;
    c = obs.c;
    rho0 = obs.rho0;
    h = obs.h;
    
    for i = 1:6
        rho(i) = sqrt((o(1,i+1) - c(1))^2 + (o(2,i+1) - c(2))^2) - R;   %this is ||O(i) - b|| described in textbook
        ox = o(1,i+1);
        oy = o(2,i+1);
        oz = o(3,i+1);
        cx = c(1);
        cy = c(2);
        % assume the robot will not go below the workspace plane
        % when sqrt((o(1,i+1) - c(1))^2 + (o(2,i+1) - c(2))^2) > R
        if (rho(i)+R) > R
            if oz <= h 
                rhoi_v = ((o(:, i+1) - [c(1); c(2); o(3, i+1)])/norm(o(:, i+1) - [c(1); c(2); o(3, i+1)])) * rho(i); %this is O(i) - b
                if rho(i) <= rho0
                    F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v)); %Calculate F_rep when ||O(i) - b|| < rho0
                else
                    F_rep(:, i) = [0;0;0];
                end
            else % oz > h but is not directly above the top surface of the cylinder
                b = [cx, cy, h] + [ox-cx, oy-cy, 0]*(1/(sqrt((ox-cx)^2+(oy-cy)^2)))*R;
                rhoi_v = [ox, oy, oz] - b;   %this is O(i) - b
                rho(i) = norm(rhoi_v);  %this is ||O(i) - b||
                if rho(i) <= rho0
                    F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v)); %Calculate F_rep when ||O(i) - b|| < rho0
                else
                    F_rep(:, i) = [0;0;0];
                end
            end
        % when sqrt((o(1,i+1) - c(1))^2 + (o(2,i+1) - c(2))^2) < R, oi is directly above the top surface of the cylinder
        else
            if o(3,i+1) > 32
                rho(i) = o(3,i+1) - h -32;
                rhoi_v = [0, 0, o(3,i+1)-h-32];
            else
                rho(i) = 32 - o(3,i+1);
                rhoi_v = [0, 0, o(3,i+1)-32];
            end
            if rho(i) <= rho0
                F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v)); 
            else
                F_rep(:, i) = [0;0;0];
            end
        end
    end
        
        
%sphere    
elseif strcmp(obs.type, 'sph')
    R = obs.R;
    c = obs.c;
    rho0 = obs.rho0;
    for i = 1:6
        rho(i) = norm(o(:,i+1) - c) - R;   %this is ||O(i) - b|| described in textbook
        rhoi_v = (o(:, i+1) - c)/norm(o(:, i+1) - c) * rho(i);  %this is O(i) - b
        if rho(i) <= rho0
            F_rep(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v)); %Calculate F_rep when ||O(i) - b|| < rho0
        else
            F_rep(:, i) = [0;0;0];
        end
    end
end

%workplane
F_rep_plane = zeros(3,1);
for i = 1:6
    if o(3,i+1) > 32
        rho(i) = o(3,i+1) - 32;
        rhoi_v = [0, 0, o(3,i+1)-32];
    else
        rho(i) = 32 - o(3,i+1);
        rhoi_v = [0, 0, o(3,i+1)-32];
    end
    
    if rho(i) <= rho0
            F_rep_plane(:, i) = zeta*((1/rho(i)) - (1/rho0)) * (1/rho(i)^2) * (rhoi_v/norm(rhoi_v)); %Calculate F_rep when ||O(i) - b|| < rho0
        else
            F_rep_plane(:, i) = [0;0;0];
    end
    
    %F_rep(:,i) = F_rep(:,i) + F_rep_plane(:, i);
end


F_rep = F_rep * 1000000;


%Calculate Jacobian
for i = 1:6
    for j = 1:i
        J_o(:,j,i) = cross(z(:,j), o(:, i+1) - o(:,j)); 
    end
end

%Calculate tau
tau = [0; 0; 0; 0; 0; 0];
for i = 1:6
   tau = tau + J_o(:,:,i).' * F_rep(:,i);
end

tau = tau.';
if norm(tau) ~= 0
    tau = tau/norm(tau); 
end

end