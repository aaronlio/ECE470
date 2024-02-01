H = 1;
q = do(H, myrobot);

function q = do(H, myrobot)
    DH = [myrobot.a' myrobot.alpha' myrobot.d'];

    Rd = H(1:3, 1:3);
    o_d_0 = H(1:3, 4);
    d_6 = DH(6, 3);

    o_c_0 = o_d_o -Rd*[0;0;1]*d_6;
    oc_x = o_c_0(1);
    oc_y = o_c_0(2);
    oc_z = o_c_0(3);

    theta1 = 0;
    theta2 = 0;
    theta3 = 0;
    

end
