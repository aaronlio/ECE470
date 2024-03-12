function myrobot = mypuma560(DH)
    % Initialize list of links
     L = [Link.empty];
    for i = 1:6
        % Create links from DH Table values
        L(i) = Link('a', DH(i, 1), 'alpha', DH(i, 2), 'd', DH(i, 3));
    end
    % Create serial link from list of links
    myrobot = SerialLink(L, 'name', 'Puma 560');
end