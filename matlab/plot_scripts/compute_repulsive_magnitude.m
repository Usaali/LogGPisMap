function y = compute_repulsive_magnitude(x, vmax, alpha, rho, useFlacco, doPlot)
% maps distances (x) to magnitude of a repulsive action (y)

if useFlacco
    disp('Using Flacco exponential function')
    y = vmax ./ ( 1 + exp( (x .* (2/rho) - 1) * alpha) );
else
    disp('Using a linear function')
    y = - vmax / rho .* x + vmax;
end

if doPlot
    figure;
    plot(x,y,'*');
end
