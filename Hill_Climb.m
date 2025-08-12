function[bPosit, bValue] = Hill_Climb(dimensions, dPosit, dValue, steps, newPol, Lb, Ub, environment)
% Hill_Climb - Scanning areas for higher pollution.
%   Get a drone to search through neighbours to find a better position:
    
    X = 1; Y = 2; Z = 3;
    bPosit = dPosit;
    bValue = dValue;
    
    % Used so drones can see at a distance:
    scanDistance = 2;

    % Shortening position coordinates:
    cX = dPosit(X);
    cY = dPosit(Y);
    cZ = dPosit(Z);

    sPosits = zeros(steps, dimensions);
    sValues = zeros(steps, 1);

    % Going through a search:
    for s = 1:steps
        % Scanning for the best neighbourhood:
        nX = (cX + (newPol(s, X) * scanDistance));
        nY = (cY + (newPol(s, Y) * scanDistance));
        nZ = (cZ + (newPol(s, Z)));
        nPos = [nX, nY, nZ];

        % Checking if the new position is inside the area:
        if (Boundary_Check(nPos, Lb, Ub) == true)
            sPosits(s, X) = nX;
            sPosits(s, Y) = nY;
            sPosits(s, Z) = nZ;
            sValues(s) = environment(nX, nY, nZ);
        else
            sPosits(s, X) = -1;
            sPosits(s, Y) = -1;
            sPosits(s, Z) = -1;
            sValues(s) = -1;
        end
        
        % Checking if the new position has the highest discovered value:
        if (sValues(s) > bValue)
            bPosit(X) = (cX + newPol(s, X));
            bPosit(Y) = (cY + newPol(s, Y));
            bPosit(Z) = (cZ + newPol(s, Z));
            bValue = environment(nX, nY, nZ);
        end
    end

end