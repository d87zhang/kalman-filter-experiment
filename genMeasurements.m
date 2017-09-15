function z = genMeasurements(measurement_sigma)

    %% declare global variables
    global dt t_f NUM_ITER n m;
    dt = 0.01;
    t_f = 1;
    NUM_ITER = t_f / dt + 1;
    n = 6; % dimension of s
    m = 2; % dimension of z
    
    torque = repmat( sin(linspace(0, 2*pi, NUM_ITER))', 1, m );
    z = torque + normrnd(0, measurement_sigma, NUM_ITER, m);
end