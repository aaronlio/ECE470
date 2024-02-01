function H = forward(joint, myrobot)
    % Initialize transformation matrix as an identity matrix
    H = eye(4);
    
    for i = 1:6
        % Get DH parameters -> Note to self: Plus or minus offset?
        theta = joint(i) + myrobot.links(i).offset;
        d = myrobot.links(i).d;
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;
        
        % Create transformation matrix for this link
        A = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
             sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0, sin(alpha), cos(alpha), d;
             0, 0, 0, 1];
             
        % Multiply current transformation matrix H with A
        H = H * A;
    end
end

