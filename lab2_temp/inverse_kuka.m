function q = inverse_kuka(H, myrobot)
    DH = [myrobot.d' myrobot.a' myrobot.alpha' myrobot.theta'];
    o_c_0 = H(1:3, 4) - H(1:3, 1:3)*[DH(6,2);0;DH(6,1)];
    %fprintf("%d\n", DH(6,1));
    oc_x = o_c_0(1);
    oc_y = o_c_0(2);
    oc_z = o_c_0(3);
    
    % compute theta 1 using formula
    theta1 = atan2(oc_y, oc_x);
    
    % compute theta 3 using formula
    D = ((DH(4,1))^2 + (DH(3,2))^2 + (DH(2,2))^2 - (oc_z - DH(1,1))^2 - (real(sqrt(oc_x^2 + oc_y^2))-DH(1,2))^2)...
        / (2 * real(sqrt((DH(4,1))^2 + (DH(3,2))^2)) * DH(2,2));
    theta3 = atan2(-D, real(sqrt(1-D^2))) - atan2(DH(3,2), DH(4,1));
    
    %D = (DH(4,1)^2 + DH(3,2)^2 + DH(2,2)^2 - (oc_z - DH(1,1))^2 - (real(sqrt(oc_x^2 + oc_y^2))-DH(1,2))^2)...
    %    / (2 * DH(2,2) * real(sqrt((DH(4,1))^2 + (DH(3,2))^2)));
    %theta3 = 3*pi/2 - atan2(-real(sqrt(1-D^2)), D) - atan2(DH(3,2), DH(4,1)); 
    
    % compute theta 2 using formula
    beta1 = atan2(oc_z-DH(1,1), real(sqrt(oc_x^2 + oc_y^2))-DH(1,2));
    beta2 = atan2(real(sqrt((DH(4,1))^2+(DH(3,2))^2))*sin(atan2(DH(3,2), DH(4,1))+theta3-pi/2) ,...
            DH(2,2)+real(sqrt((DH(4,1))^2+(DH(3,2))^2))*cos(atan2(DH(3,2), DH(4,1))+theta3-pi/2));
    
    theta2 = beta1 - beta2;

    % Compute R03 using forward function
    R03 = forward_kuka([theta1 theta2 theta3], myrobot);
    
    % Find R36 
    R36 = R03(1:3, 1:3).' * H(1:3, 1:3);

    % Extract theta4, theta5, theta6
    theta4 = atan2(R36(2,3), R36(1,3));
    theta5 = atan2(real(sqrt(1-R36(3,3)^2)), R36(3,3));
    theta6 = atan2(R36(3,2), -R36(3,1));
    
    %theta4 = atan2(-R36(2,3), -R36(1,3));
    %theta5 = atan2(-real(sqrt(1-R36(3,3)^2)), R36(3,3));
    %theta6 = atan2(-R36(3,2), R36(3,1));

    q = [theta1 theta2 theta3 theta4 theta5 theta6];
    
    %wrap angles around [pi, -pi)
    for i = 1:3
        temp = mod(q(i), 2*pi);
        if temp > pi
            temp = temp - 2*pi;
        elseif temp < -pi
            temp = temp + 2*pi;
        end
        q(i) = temp;
    end
    
end
