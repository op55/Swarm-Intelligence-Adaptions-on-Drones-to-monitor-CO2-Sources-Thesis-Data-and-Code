function[drones, locations] = Place_Drones(iter, numDrones, X, Y, Z, baseSize)
% Place_Drones - Setting up the drones.
%   Placing the drones inside their base:

    %% Variables:

    locations = zeros(X, Y, Z);         % Using the area for locations.
    droneBase = (X - (Y - baseSize));   % The size of the drone base.

    coords = zeros(numDrones, 3);       % The coordinates for each drone.
    values = zeros(numDrones, 1);       % The current value for each drone.
    velocs = zeros(numDrones, 1);       % The current velo for each drone.

    xPos = floor(droneBase / 2);        % Place drones in the X's middle.
    yPos = floor(Y / (numDrones + 1));  % Evens the drone from the Y axis.

    %% Setting the base in Locations:

    % Going through all the coordinates in the drone base:
    % (The Z axis is ignored as only the bottom is used.)
    for i = 1:X
        for j = 1:droneBase
            locations(i, j, 1) = -2;
        end
    end

    %% Placing the drones:

    for i = 1:numDrones
        % Setting the start position for each drone:
        newX = (yPos * i);    newY = xPos;          newZ = 1;
        coords(i, 1) = newX;  coords(i, 2) = newY;  coords(i, 3) = newZ;
        velocs(i, 1) = 0;     velocs(i, 2) = 0;     velocs(i, 3) = 0;
        values(i) = 0;

        % Setting up a drone's location:
        locations(newX, newY, newZ) = i;
    end

    drones = Drone(iter, numDrones, values, coords, velocs);

end