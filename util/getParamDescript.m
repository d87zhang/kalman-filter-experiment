function descript = getParamDescript(param_idx, EST_CENTER_OF_MASS_ALONE)
    % Get description for a param index assuming a 10n parameter structure

    joint_num = floor((param_idx-1)/10) + 1;
    
    type_idx = mod(param_idx, 10);
    
    switch type_idx
        case 1
            type_str = 'mass';
        case 2
            if EST_CENTER_OF_MASS_ALONE
                type_str = 'CoM_x';
            else
                type_str = 'CoM_x * m';
            end
        case 3
            if EST_CENTER_OF_MASS_ALONE
                type_str = 'CoM_y';
            else
                type_str = 'CoM_y * m';
            end
        case 4
            if EST_CENTER_OF_MASS_ALONE
                type_str = 'CoM_z';
            else
                type_str = 'CoM_z * m';
            end
        case 5
            type_str = 'I_xx';
        case 6
            type_str = 'I_yy';
        case 7
            type_str = 'I_zz';
        case 8
            type_str = 'I_xy';
        case 9
            type_str = 'I_yz';
        case 0
            type_str = 'I_xz';
    end
    
    descript = sprintf('L%d - %s', joint_num, type_str);
end

