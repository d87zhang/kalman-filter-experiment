function puma = buildPumaDH(s)
    % adapted from the mdl_puma560akb.m script provided The Robotics
    % Toolbox by Peter I. Corke. Note that it builds a SerialLink object
    % following the DH convention.
    
    % his license stuff is copied below
    % Copyright (C) 1993-2015, by Peter I. Corke
    %
    % This file is part of The Robotics Toolbox for MATLAB (RTB).
    % 
    % RTB is free software: you can redistribute it and/or modify
    % it under the terms of the GNU Lesser General Public License as published by
    % the Free Software Foundation, either version 3 of the License, or
    % (at your option) any later version.
    % 
    % RTB is distributed in the hope that it will be useful,
    % but WITHOUT ANY WARRANTY; without even the implied warranty of
    % MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    % GNU Lesser General Public License for more details.
    % 
    % You should have received a copy of the GNU Leser General Public License
    % along with RTB.  If not, see <http://www.gnu.org/licenses/>.
    %
    % http://www.petercorke.com
    
    %            theta    d        a    alpha
    L(1) = Link([  0      0        0       pi/2    0], 'standard');
    L(2) = Link([  0      0        0.4318  0       0], 'standard');
    L(3) = Link([  0      0.15005  0.0203 -pi/2    0], 'standard');
    L(4) = Link([  0      0.4318   0       pi/2    0], 'standard');
    L(5) = Link([  0      0        0      -pi/2    0], 'standard');
    L(6) = Link([  0      0        0       0       0], 'standard');

    % specification for s
    % s = [l1_m,
    %      l1_r_x, l1_r_y, l1_r_z,
    %      l1_Ixx, l1_Iyy, l1_Izz, l1_Ixy, l1_Iyz, l1_Ixz,
    %      ... so on for links 2 to n (10 params per link)];
    
    for idx = 1:(length(s)/10)
        base_idx = 10 * (idx-1);
        L(idx).m = s(base_idx + 1);
        
        if L(idx).m ~= 0
            L(idx).r = s(base_idx + 2:base_idx + 4) / L(idx).m;
        else
            L(idx).r = zeros(1, 3);
        end
        
        L(idx).I = s(base_idx + 5:base_idx + 10);
    end
    
    L(4).m = 0.82;
    L(5).m = 0.34;
    L(6).m = .09;

    %         rx      ry      rz
    L(4).r = [0,  0.019, 0];
    L(5).r = [0   0   0 ];
    L(6).r = [0   0   0.032  ];

    %        Ixx     Iyy      Izz    Ixy     Iyz     Ixz
    L(4).I = [1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0];
    L(5).I = [0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0];
    L(6).I = [0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0];

    L(1).Jm =  200e-6;
    L(2).Jm =  200e-6;
    L(3).Jm =  200e-6;
    L(4).Jm =  33e-6;
    L(5).Jm =  33e-6;
    L(6).Jm =  33e-6;

    L(1).G =  -62.6111;
    L(2).G =  107.815;
    L(3).G =  -53.7063;
    L(4).G =  76.0364;
    L(5).G =  71.923;
    L(6).G =  76.686;

    % viscous friction (motor referenced)
    % unknown

    % Coulomb friction (motor referenced)
    % unknown

    %
    % some useful poses
    %
%     qz = [0 0 0 0 0 0]; % zero angles, L shaped pose
%     qr = [0 -pi/2 pi/2 0 0 0]; % ready pose, arm up
%     qstretch = [0 0 pi/2 0 0 0]; % horizontal along x-axis

    puma = SerialLink(L, 'name', 'Puma560-AKB', 'manufacturer', 'Unimation', 'comment', 'AK&B');
end