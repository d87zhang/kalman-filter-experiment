% initialize different experimental setups (sets of parameters required for
% an experiment

traj_coef_file = matfile('coef.mat');
BOOL_TO_STRING = {'false', 'true'};

clear SPONG_PLANE_MAN_SETUP;

EST_CENTER_OF_MASS_ALONE = false;

SPONG_PLANE_MAN_SETUP.NUM_JOINTS = 2;
SPONG_PLANE_MAN_SETUP.n = 2 * 10;
SPONG_PLANE_MAN_SETUP.m = 2;
SPONG_PLANE_MAN_SETUP.L1 = 2;
SPONG_PLANE_MAN_SETUP.L2 = 2;
% use frame i for reference frame for center of mass i, 'cause the other
% way is broken
% SPONG_PLANE_MAN_SETUP.robot_set_params_func = @(robot, s)(setSpongPlaneParams(robot, s, L1, L2));
% SPONG_PLANE_MAN_SETUP.robot_set_param_func = @(robot, s_value, idx)(setSpongPlaneParam(robot, s_value, idx, L1, L2));
SPONG_PLANE_MAN_SETUP.robot_set_params_func = @(robot, s)(setFullRoboParams(robot, s, EST_CENTER_OF_MASS_ALONE));
SPONG_PLANE_MAN_SETUP.robot_set_param_func = @(robot, s_value, idx)(setFullRoboParam(robot, s_value, idx, EST_CENTER_OF_MASS_ALONE));
SPONG_PLANE_MAN_SETUP.robot_build_func = @(s)(buildSpongPlaneMan(...
    s, SPONG_PLANE_MAN_SETUP.L1, SPONG_PLANE_MAN_SETUP.L2, ...
    SPONG_PLANE_MAN_SETUP.robot_set_params_func));

SPONG_PLANE_MAN_SETUP.s_actual = zeros(SPONG_PLANE_MAN_SETUP.n, 1);
SPONG_PLANE_MAN_SETUP.s_actual(1:10) = [1, -1, 0, 0, ...
                                        0, 0, 1, 0, 0, 0];
SPONG_PLANE_MAN_SETUP.s_actual(11:20) = [1, -1, 0, 0, ...
                                         0, 0, 1, 0, 0, 0];
SPONG_PLANE_MAN_SETUP.assumed_measurement_sigma = [1, 0.5];

SPONG_PLANE_MAN_SETUP.traj_coef = traj_coef_file.ff_coef_plane;

SPONG_PLANE_MAN_SETUP.theta_func = @(s)(calcThetaForSpongPlaneMan(s, ...
    EST_CENTER_OF_MASS_ALONE, SPONG_PLANE_MAN_SETUP.L1, SPONG_PLANE_MAN_SETUP.L2));

clear DH_PUMA_EST_3L_SETUP;

DH_PUMA_EST_3L_SETUP.NUM_JOINTS = 6;
DH_PUMA_EST_3L_SETUP.n = 3 * 10;
DH_PUMA_EST_3L_SETUP.m = 3;
DH_PUMA_EST_3L_SETUP.robot_build_func = @buildPumaDH;
DH_PUMA_EST_3L_SETUP.robot_set_params_func = @setFullRoboParams;
DH_PUMA_EST_3L_SETUP.robot_set_param_func = @setFullRoboParam;

DH_PUMA_EST_3L_SETUP.s_actual = zeros(DH_PUMA_EST_3L_SETUP.n, 1);
DH_PUMA_EST_3L_SETUP.s_actual(1:10) = [0, 0, 0, 0, 0   0.35   0   0   0   0];
DH_PUMA_EST_3L_SETUP.s_actual(11:20) = [17.4, 17.4 * -0.3638, 17.4 * 0.006, 17.4 * 0.2275, ...
                                        0.13, 0.524, 0.539, 0, 0, 0];
DH_PUMA_EST_3L_SETUP.s_actual(21:30) = [4.8, 4.8 * -0.0203, 4.8 * -0.0141, 4.8 * 0.070, ...
                                        0.066, 0.086, 0.0125, 0, 0, 0];
% s_actual(31:40) = [0.82, 0, 0.82 * 0.019, 0, ...
%                    1.8e-3, 1.3e-3, 1.8e-3, 0, 0, 0];
% s_actual(41:50) = [0.34, 0, 0, 0, ...
%                    0.3e-3, 0.4e-3, 0.3e-3, 0, 0, 0];
% s_actual(51:60) = [0.09, 0, 0, 0.09 * 0.032, ...
%                    0.15e-3, 0.15e-3, 0.04e-3, 0, 0, 0];
DH_PUMA_EST_3L_SETUP.assumed_measurement_sigma = [2, 3, 1.5];
% DH_PUMA_EST_3L_SETUP.assumed_measurement_sigma = [2, 3, 1.5, 0.1, 0.1, 0.1]

DH_PUMA_EST_3L_SETUP.traj_coef = traj_coef_file.ff_coef2;


clear SIMPLE_PLANE_MAN_SETUP;

SIMPLE_PLANE_MAN_SETUP.NUM_JOINTS = 2;
SIMPLE_PLANE_MAN_SETUP.n = 2;
SIMPLE_PLANE_MAN_SETUP.m = 2;

S_SPEC = 'mass';
SIMPLE_PLANE_MAN_SETUP.robot_build_func = @(s)(buildPlaneMan(s, S_SPEC));
SIMPLE_PLANE_MAN_SETUP.robot_set_params_func = @(robot, s)(setPlaneParams(robot, s, S_SPEC));
SIMPLE_PLANE_MAN_SETUP.robot_set_param_func = @(robot, s_value, idx)(setPlaneParam(robot, s_value, idx, S_SPEC));

SIMPLE_PLANE_MAN_SETUP.s_actual = [1.5, 2]';
SIMPLE_PLANE_MAN_SETUP.assumed_measurement_sigma = [6, 4];

SIMPLE_PLANE_MAN_SETUP.traj_coef = traj_coef_file.ff_coef_plane;

% maybe for later
% assumed_measurement_sigma = 1 * [2, 3, 1.5, 0.1, 0.1, 0.1];
