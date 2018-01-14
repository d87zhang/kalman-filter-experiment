%% the setup
% robot is the two link, revolute joint, planar robot from the Spong book.
% See equations on p.270

t = sym('t', 'real');
syms q1 q2;

q1 = t;
q2 = t^2;
% q1 = t^2;
% q2 = t^2;
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

integrand = simplify(Y' * Y);

if NUMERICAL_APPROACH
    fun_to_integrate = @(t_val)(eval(subs(integrand, t, t_val)));
    obs_gramiam = integral(fun_to_integrate, t_0, t_f, 'ArrayValued', true);
    % TODO outdated?
else
    obs_gramiam = int(integrand, t);
    obs_gramiam = simplify(obs_gramiam);
end

% Fix integrations that Matlab can't handle
% obs_gramiam(2,2) = 19/2 * sqrt(sym(pi)) * fresnelc((2*t)/sqrt(sym(pi))) ...
%                     + t*(cos(2*t^2) + 4*(5 + 4*t^4 - 5*t^2*sin(2*t^2)));

% evaluating the definite integral..
t_0 = 0;
t_f = 1;
def_integral = subs(obs_gramiam, t, t_f) - subs(obs_gramiam, t, t_0);
def_integral = simplify(def_integral);

det(def_integral)
rank(def_integral)
