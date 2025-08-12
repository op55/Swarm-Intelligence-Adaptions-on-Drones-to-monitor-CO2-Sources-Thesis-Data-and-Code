function[drone, locations] = Update_Map(drone, locations)
% Update_Map - Updating a drone's position.
%   Changing a drone's visual location and reflecting it on the map:
    
    X = 1; Y = 2; Z = 3;

    % Getting the drone's new position:
    newPos = drone.POSIT;
    prevPos = drone.PREV_POSIT;

    % Referencing the coordinates of the new position:
    nX = newPos(X); nY = newPos(Y); nZ = newPos(Z);

    % Referencing the coordinates of the old position:
    pX = prevPos(X); pY = prevPos(Y); pZ = prevPos(Z);

    % Assign a new entries if the drone is not waiting in a flag:
    if (drone.MADE_FLAG == false)
        % Updating the drone's location in the area:
        locations(nX, nY, nZ) = drone.ID;
        locations(pX, pY, pZ) = 0;        
    end

end