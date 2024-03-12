function qref = motionplan(q0, q2, t1, t2, myrobot, obs, tol)
alpha = 0.01;
q = q0;
while norm(q(size(q,1), 1:5) - q2(1:5)) > tol
    q_kp1 = q(size(q,1), :) + alpha*att(q(size(q,1), :), q2, myrobot);
    q = [q;q_kp1];
end
q(:, 6) = linspace(q0(6), q2(6), size(q, 1)); 

t = linspace(t1, t2, size(q,1));
qref = spline(t,q');
end