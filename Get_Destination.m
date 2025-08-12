function[velocities] = Get_Destination(dimensions, dPos, oPos, upMove, Z)
% Get_Destination - Getting the velocities for a destination.
%   Getting the velocities for a drone who's in the the 'Approach' role:
    
    % The velocities to return to the drone:
    velocities = zeros(1, dimensions);

    % Used to ensure one dimension is moving the max velocity:
    maxMove = randi(dimensions-1);
    
    % Determining the drone's velocity for this iteration:
    for m = 1:(dimensions-1)
        % Getting the velocity for the X or Y axis:
        velocities(m) = dPos(m) - oPos(m);
    
        % Restricting the movement of dimensions, based
        % on the boundaries of the upper and lower bounds:
        if (velocities(m) > upMove(m))
            if (maxMove == m); velocities(m) = upMove(m);
            else; velocities(m) = randi([1, upMove(m)]); end
        elseif (velocities(m) < -upMove(m))
            if (maxMove == m); velocities(m) = -upMove(m);
            else; velocities(m) = randi([-upMove(m), -1]); end
        end
    end
    
    % Limiting movements for the Z axis:
    velocities(Z) = dPos(Z) - oPos(Z);
    
    % Having smaller boundaries for the Z axis:
    if (velocities(Z) > upMove(Z)); velocities(Z) = randi([0, upMove(Z)]);
    elseif (velocities(Z) < -upMove(Z)); velocities(Z) = randi([-upMove(Z), 0]); end

end

