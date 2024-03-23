function myrobot = mykuka(DH)
     L = [Link.empty];
    for i = 1:6
        L(i) = Link('d', DH(i, 2), 'a', DH(i, 3), 'alpha', DH(i, 4));
    end
    myrobot = SerialLink(L, 'name', 'Puma 560');
end