function qref = motionplan(q0, q2, t1, t2, myrobot, obs, tol)
alpha_att = 0.013;
alpha_rep = 0.01;
q = q0;
max_loop = 3000;
index = 0;

while true
   %calculate the sum of attractive and repulsive tau
   tau_rep = [0,0,0,0,0,0];
   for i = 1:size(obs)
      tau_rep = tau_rep +  rep(q(size(q,1), :), myrobot, obs{i});
   end
   tau_att = att(q(size(q,1), :), q2, myrobot);
   q_kp1 = q(size(q,1),:) + alpha_att*tau_att + alpha_rep*tau_rep;
   q = [q;q_kp1];
    
   %wrap angles aroung 2pi for termination comparison
   new_q = zeros(1,5);
   target_q = q2(1:5);
   for i = 1:5
        new_q(i) = new_q(i) - 2*pi*floor(new_q(i)/(2*pi));
        target_q(i) = target_q(i) - 2*pi*floor(target_q(i)/(2*pi));
   end
   
   %termination
   if norm(new_q - q2(1:5)) < tol || index >= max_loop
       break;
   end
   index = index + 1;
end

%produce q6
q(:, 6) = linspace(q0(6), q2(6), size(q, 1)); 

%create cubic spline
t = linspace(t1, t2, size(q,1));
qref = spline(t,q');
end


