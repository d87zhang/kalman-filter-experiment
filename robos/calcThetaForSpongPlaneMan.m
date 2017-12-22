function theta = calcThetaForSpongPlaneMan(s, EST_CENTER_OF_MASS_ALONE, L1, L2)
    % Returns the value of the minimal parameter set as a column vector

    m1 = s(1);
    I1 = s(7);
    m2 = s(11);
    I2 = s(17);
    if ~EST_CENTER_OF_MASS_ALONE
        rx1 = s(2) / m1;
        rx2 = s(12) / m2;
    else
        rx1 = s(2);
        rx2 = s(12);
    end
    lc1 = rx1 + L1;
    lc2 = rx2 + L2;
    
    theta = zeros(5, 1);
    theta(1) = m1*lc1^2 + m2*(L1^2 + lc2^2) + I1 + I2;
    theta(2) = m2*L1*lc2;
    theta(3) = m2*lc2^2 + I2;
    theta(4) = m1*lc1 + m2*L1;
    theta(5) = m2*lc2; % the textbook has a typo...
end

