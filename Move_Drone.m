function[drone] = Move_Drone(dimensions, drone, Ub, Lb)
% Move_Drone - Moving a drone:
%   Commencing a drone to move, based on its new velocity.

    % Going through the number of dimensions:
    for m = 1:dimensions      
        % Unique variables:
        drone.POSIT(m) = drone.POSIT(m) + drone.VELOCITIES(m);

        % Commencing error handling in case of 'out of bounds' scenarios:
        if (drone.POSIT(m) > Ub(m)); drone.POSIT(m) = floor(Ub(m) - 1);
        elseif (drone.POSIT(m) < Lb(m)); drone.POSIT(m) = floor(Lb(m) + 1); end
    end
end