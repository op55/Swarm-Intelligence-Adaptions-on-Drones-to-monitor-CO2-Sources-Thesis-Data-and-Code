function[drone] = Battery_Deplete(drone, dimensions, ratio, lowBattery)
% Deplete_Battery - Decrease a drone's battery.
%   Deplete a drone's battery, taking actions if it hits low levels:

    % Used to determine the total spaces moved in an iteration:
    tMove = 1;

    % Shortening the drone's position coordinates:
    newPos  = drone.POSIT;
    prevPos = drone.PREV_POSIT;
    
    % Calculating how much spaces the drone has moved by:
    for m = 1:dimensions
        if (newPos(m) > prevPos(m))
            tMove = tMove + (newPos(m) - prevPos(m));
        else
            tMove = tMove + (prevPos(m) - newPos(m));
        end
    end
    
    % Decreasing the drone's battery life based on movement:
    drone.BATTERY = drone.BATTERY - (ratio * tMove);
    
    % Checks if the drone's battery has depleted:
    if (drone.BATTERY <= 0)
        drone.STATUS = "DEAD";
        drone.FINISHED = true;
    end
    
    % Checking if the drone's battery is at a low level:
    if (drone.ACTIVE == true)
        if (drone.BATTERY < lowBattery)
            % Setting the drone's X and Y start
            % points to be its new destination:
            for m = 1:(dimensions-1)
                drone.DESTINATION(m) = drone.START_POSIT(m);
            end
    
            % Setting the velocity for the Z axis to 0, to
            % avoid the drone going out of the search area:
            drone.DESTINATION(dimensions) = drone.POSIT(dimensions);

            % Set the drone's current position as its last one for memory:
            drone.LAST_POSIT = drone.POSIT;

            % Resetting the drone's disabled time:
            drone.DISABLED_STEPS = 0;
            
            % Will remember to continue scanning once battery is refilled:
            if (drone.ZZC_ROLE == "SCAN"); drone.NEW_SEARCH = false; end
    
            % Getting the drone to return to base:
            drone.STATUS = "RETURN";
            drone.ACTIVE = false;
        end       
    end
    
    % Making the drone's previous position its current one:
    drone.PREV_POSIT = drone.POSIT;

    % Incrementing the drone's total distance:
    drone.TOTAL_DISTANCE = drone.TOTAL_DISTANCE + (tMove - 1);
end