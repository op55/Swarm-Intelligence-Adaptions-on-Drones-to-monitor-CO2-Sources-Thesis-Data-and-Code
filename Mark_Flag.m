function[states, flags] = Mark_Flag(states, flags, drone, radius, Lb, Ub, locked, flagID, steps, timer)
% Mark_Flag - Marking a flag for the system.
%   Adding a flag to the database, and locking down the area around it:
    
    posit          = drone.POSIT;
    value          = drone.VALUE;
    droneID        = drone.ID;
    droneBatteries = drone.BATTERY_PACKS;

    nX = posit(1); nY = posit(2); nZ = posit(3);

    % Lock the radius around the flag:
    for x = -radius:radius
        for y = -radius:radius
            for z = -radius:radius
                lX = (nX + x);
                lY = (nY + y);
                lZ = (nZ + z);
                lPos = [lX, lY, lZ];
    
                % Check position is in boundaries:
                if (Boundary_Check(lPos, Lb, Ub) == true)
                    if (states(lX, lY, lZ) ~= locked)
                        states(lX, lY, lZ) = locked;
                    end
                end
            end
        end
    end
    
    % Add a new flag in database:
    flags(flagID) = Pollution_Flag(flagID, posit, value, steps, timer, droneID, droneBatteries);
    
end