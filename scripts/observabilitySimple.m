%% the setup
% robot is the two link, revolute joint, planar robot from the Spong book.
% See equations on p.270

syms t q1 q2;

% TODO try sin or cos
q1 = t^2;
q2 = t^2;
% q1 = sin(t);
% q2 = sin(t);

NUMERICAL_APPROACH = false;

qd1 = diff(q1);
qd2 = diff(q2);
qdd1 = diff(qd1);
qdd2 = diff(qd2);

%% the analysis
% gravity-free equation taken from p.270
Y = [qdd1, cos(q2)*(2 * qdd1 + qdd2) - sin(q2)*(qd1^2 + 2 * qd1 * qd2), qdd2; ...
     0, cos(q2) * qdd1 + sin(q2) * qd1^2, qdd1 + qdd2];

% indef_integral = int(Y.' * Y, t); % no closed-form solution, at least according to MatLab

% evaluating the definite integral..
t_0 = 0;
t_f = 10;

if NUMERICAL_APPROACH
    fun_to_integrate = @(t_val)(eval(subs(Y.' * Y, t, t_val)));
    obs_gramiam = integral(fun_to_integrate, t_0, t_f, 'ArrayValued', true);
else
    obs_gramiam = int(Y.' * Y, t);
    % TODO implement (if used at all..)
end

det(obs_gramiam)
rank(obs_gramiam)
