function q = inverse(H, myrobot)

    DH = [ myrobot.a' myrobot.alpha' myrobot.d' myrobot.theta'];
    o_c_0 = H(1:3, 4) - H(1:3, 1:3) *[0;0;1]*DH(6,3);
    oc_x = o_c_0(1);
    oc_y = o_c_0(2);
    oc_z = o_c_0(3);

    theta1 = atan2(oc_y, oc_x) - atan2(-DH(2,3), real(sqrt(oc_x^2 + oc_y^2 - DH(2,3)^2)));
    

    D = (((oc_z -DH(1,3))^2 + oc_x^2 + oc_y^2  - DH(2,3)^2) - DH(2,1)^2 - DH(4,3)^2) / (2 * DH(2,1) * DH(4,3));
    theta3 = atan2(D, real(sqrt(1-D^2))); 
    
    theta2 = atan2(oc_z - DH(1, 3), real(sqrt(oc_x^2 + oc_y^2 -DH(2,3)^2))) - ...
        atan2(DH(4,3)*sin(theta3-(pi/2)), DH(2,1) + DH(4,3)*cos(theta3-(pi/2)));

    R03 = forward([theta1 theta2 theta3], myrobot);
    R36 = R03(1:3, 1:3).' * H(1:3, 1:3);

    theta4 = atan2(R36(2,3), R36(1,3));
    theta5 = atan2(real(sqrt(1-R36(3,3)^2)), R36(3,3));
    theta6 = atan2(R36(3,2), -R36(3,1));

    q = [theta1 theta2 theta3 theta4 theta5 theta6];

end
