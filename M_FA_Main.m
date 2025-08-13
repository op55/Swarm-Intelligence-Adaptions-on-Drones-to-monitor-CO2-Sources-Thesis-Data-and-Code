function[] = M_FA_Main(parameters, environment, drones, locations, run)
    
    X = 1;  Y = 2;  Z= 3;

    %% Getting the parameters into their own variables:
    
    GENERAL            = parameters{1};
    iterations         = GENERAL(1);
    dimensions         = GENERAL(2);
    numDrones          = GENERAL(3);
    
    SEARCHES           = parameters{2};
    minAltitude        = SEARCHES(1);
    stateCheck         = SEARCHES(2);
    
    SCENARIOS          = parameters{3};
    minPollution       = SCENARIOS(1);
    sevPollution       = SCENARIOS(2);
    decayRate          = SCENARIOS(3);
    windInfluence      = SCENARIOS(4);

    BATTERIES          = parameters{4};
    lowBattery         = BATTERIES(1);
    totalBatteries     = BATTERIES(2);
    decreaseRatio      = BATTERIES(3);
    replaceTime        = BATTERIES(4);
    
    areas              = parameters{5};
    smokes             = parameters{6};
    scales             = parameters{7};
    
    PLUMES             = parameters{8};
    originPos          = parameters{9};
    ppmMax             = PLUMES(1,:);
    pStartTimes        = PLUMES(2,:);
    pStopTimes         = PLUMES(3,:);
    pMinMove           = PLUMES(4,:);
    pSpreads           = parameters{10};
    pVelocities        = parameters{11};
    
    WINDS              = parameters{12};
    initialWinds       = WINDS(1,:);
    finalWinds         = WINDS(2,:);

    TRANSITIONS        = parameters{13};
    transitionStart    = TRANSITIONS(1);
    transitionDuration = TRANSITIONS(2);

    MOVEMENTS          = parameters{14};
    upMove             = MOVEMENTS(1,:);
    loMove             = MOVEMENTS(2,:);
    

    %% M-FA Variables:
    
    % RD-ABC Values:
    alpha = 1;   % Randomness factor
    theta = 0.97;  % The decrease rate of alpha.

    % Attributes for the flag areas:
    radius = 1;                 % How big the radius of a flagged area.
    locked = -1;                % Indicates a locked coordinate.
    flagMade = false;           % What to do once all drones have arrived.
    flagRadius = [25, 25, 2];   % Size of source flag radius when finished.

    % Attributes for drone counting:
    dronesArrived = 0;      % How many drones have reached their destination.
    dronesInBase = 0;       % How many drones need to be at their base.
    dronesFinished = 0;     % How many drones have finished.
    noDronesActive = false; % Determines whether at least one drone is active.
    
    % Storing each dimenion into their own variable:
    areaX = areas(1);   areaY = areas(2);   areaZ = areas(3);

    % The boundaries for each dimension:
    Ub = [areaX, areaY, areaZ];
    Lb = [1, 1, minAltitude];


    %% Pollution Flag Variables:

    % Entries to store for outputs:
    totalFlags = 1;         % For storing the total peaks.

    % Storing each pollution, starting with an empty one for a placeholder:
    polFlags = Pollution_Flag.empty(0, numel(1));

    % Used for reference when marking pollution flags in the area:
    polStates = zeros(areaX, areaY, areaZ);

    
    %% Running M-FA:

    % Starting the timer:
    timer = 0;  tic;

    for iter = 1:iterations

        % Update environment at each time step
        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );

        % Iterating through the number of drones:
        for d = 1:numDrones

            % Checks if the current drone has finished:
            if (drones(d).FINISHED == true); continue; end

            % Decreasing the drone's battery life, if it is in use:
            if (drones(d).STATUS ~= "REPLACING")
                drones(d) = Battery_Deplete(drones(d), dimensions, decreaseRatio, lowBattery);
            end

            % Determining counts if a drone goes back to base or is dead:
            if (drones(d).ACTIVE == false)
                if (drones(d).NEEDS_RETURN == false)
                    dronesInBase = (dronesInBase + 1);
                    drones(d).NEEDS_RETURN = true;
                end

            end

            if (drones(d).FINISHED == true); dronesFinished = (dronesFinished + 1); end

            status = drones(d).STATUS;
            sPos = drones(d).START_POSIT;

            %% The actions which are related to the drone base:

            % The droneis currently waiting and can take no actions:
            if (status == "WAITING")
                % Skip the rest of the actions for a drone:
                continue;
            end


            % The drone is taking off from its base:
            if (status == "TAKEOFF")
                % Moving one space higher than the current Z axis:
                drones(d).POSIT(Z) = (drones(d).POSIT(Z) + 1);
                nPos = drones(d).POSIT;

                % Assigning the position's value to the drone:
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Commence M-FA role once desired altitude is reached:
                if (drones(d).POSIT(Z) == minAltitude)

                    % Used to determine the drone's M-FA role:
                    gatherCheck = false;

                    % Check if any drones are trying to gather to a place:
                    for g = 1:numDrones
                        if (drones(g).STATUS == "M-FA" && drones(g).M_ROLE == "GATHER")
                            gatherCheck = true; break;
                        end
                    end

                    % Checking what role this drone needs to be in:
                    if (gatherCheck == true)
                        % This drone needs to go to a place:
                        drones(d).M_ROLE = "GATHER";
                    else
                        % This drone should try anf find a new place:
                        drones(d).M_ROLE = "SELECT";
                    end

                    drones(d).STATUS = "M-FA";
                end

                % Skip the rest of the actions for a drone:
                continue;
            end


            % The drone is returning back above its base:
            if (status == "RETURN")
                % Shortening some variables:
                dPos = drones(d).DESTINATION;
                oPos = drones(d).POSIT;

                % Making movements for each dimension:
                drones(d).POSIT = Return_To_Base(dimensions, dPos, oPos, upMove, Ub, Lb, areas);
                nPos = drones(d).POSIT;

                % Assigning the position's value to the drone:
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Checking if the drone's has reaching its starting point:
                if (nPos(X) == sPos(X) && nPos(Y) == sPos(Y))
                    drones(d).STATUS = "LANDING";
                end

                % Skip the rest of the actions for a drone:
                continue;
            end


            % The is drone is landing on its base:
            if (status == "LANDING")

                % Moving the drone so its Z axis is one position lower:
                drones(d).POSIT(Z) = (drones(d).POSIT(Z) - 1);
                nPos = drones(d).POSIT;

                % Assigning the position's value to the drone:
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Checking if the drone has landed back in its base:
                if (nPos(Z) == sPos(Z))

                    % Stop functioning once many iterations are done:
                    if (iter > iterations * 0.95)
                        % Declaring the drone as finished:
                        drones(d).FINISHED = true;
                        dronesFinished = dronesFinished + 1;
                        continue;
                    end

                    % Shortening the name for batteries a drone has used:
                    batteries = drones(d).BATTERY_PACKS;

                    % Checking if the drone has any batteries left:
                    if (batteries < totalBatteries)
                        % Getting the drone to replace its battery:
                        drones(d).BATTERY_PACKS = (batteries + 1);
                        drones(d).REPLACE_PHASE = replaceTime;
                        drones(d).STATUS = "REPLACING";
                    else
                        % Declaring the drone as finished:
                        drones(d).FINISHED = true;
                        dronesFinished = dronesFinished + 1;
                    end
                end

                % Skip the rest of the actions for a drone:
                continue;
            end

            if (status == "REPLACING")
                % Reducing the drone's battery replacement steps:
                drones(d).REPLACE_PHASE = (drones(d).REPLACE_PHASE - 1);

                % Checking if the replacement is finished:
                if (drones(d).REPLACE_PHASE == 0)
                    % The drone resets its behaviour, for now:
                    dronesInBase = (dronesInBase - 1);
                    drones(d).STATUS = "TAKEOFF";
                    drones(d).ACTIVE = true;
                    drones(d).NEEDS_RETURN = false;
                    drones(d).BATTERY = 100.0;
                end

                continue;
            end


            %% The M-FA algorithm's actions:

            % The drone is currently trying to do M-FA:
            if (status == "M-FA")
                % Getting the drone's current M-FA action:
                role = drones(d).M_ROLE;

                % The drone wants to use the M-FA's velocity calculation:
                if (role == "GATHER")
                    % Shortening some drone-related variables:
                    dVal = drones(d).VALUE;
                    dPos = drones(d).POSIT;
                    dVelocities = drones(d).VELOCITIES;

                    for j = 1:numDrones
                        
                        jVal = drones(j).VALUE;
                        jPos = drones(j).POSIT;
                        
                        if (jVal > dVal)

                            % Calculating a velocity for the drone:
                            drones(d).VELOCITIES = Get_Velocity(dimensions, alpha, dVelocities, dPos, jPos, upMove, loMove);

                            % Move drone towards a brighter light:
                            drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);

                        end                   

                    end
                    

                    % Getting the drone's new position:
                    nPos = drones(d).POSIT;
                    nX = nPos(X); nY = nPos(Y); nZ = nPos(Z);
                    
                    % Assigning the position's value to the drone:
                    drones(d).VALUE = environment(nX, nY, nZ);

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);

                    % Checking if the drone's position has high pollution:
                    if (drones(d).VALUE > minPollution)
                        % Flagging the polluted area, if it lacks a flag:
                        if (polStates(nX, nY, nZ) > -1)

                            % Making a new flag:
                            timer = timer + toc;
                            polFlags = Mark_Flag(polFlags, drones(d), iter, timer, totalFlags);
                            tic;

                            % Locking down the flagged area:
                            polStates = Lock_Area(polStates, drones(d), radius, locked, Ub, Lb);

                            % Incrementing the total number of flags:
                            totalFlags = totalFlags + 1;
                            % Setting the drone to escape the flag area:
                            drones(d).M_ROLE = "ESCAPE";
                        end
                    end

                    % Skip the rest of the actions for a drone:
                    continue;
                end


                % The drone need to pick a destination:
                if (role == "SELECT")
                    % Variables used to determine the drone's new position:
                    destination = zeros(1, dimensions);

                    % Defining the destination position, expect the Z axis:
                    for m = 1:(dimensions-1); destination(m) = randi([1, areaX]); end

                    % Picking a random place for the accepted Z axis:
                    destination(dimensions) = randi([minAltitude, areaZ]);

                    % Assigning the new dimensional positions to a drone:
                    drones(d).DESTINATION = destination;

                    % Letting the drone approach its destination for a time:
                    drones(d).M_ROLE = "APPROACH";

                    % Skip the rest of the actions for a drone:
                    continue;
                end


                % The drone is trying to approach its destination:
                if (role == "APPROACH")
                    % Getting the drone's destination coordinates:
                    dPos = drones(d).DESTINATION;
                    dX = dPos(X); dY = dPos(Y); dZ = dPos(Z);

                    % Getting the velocities for the drone:
                    drones(d).VELOCITIES = Get_Destination(dimensions, dPos, drones(d).POSIT, upMove, Z);

                    % Moving a drone with the 'Move_Drone' function:
                    drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);

                    % Getting the drone's new position:
                    nPos = drones(d).POSIT;
                    nX = nPos(X); nY = nPos(Y); nZ = nPos(Z);

                    % Assigning the position's value to the drone:
                    drones(d).VALUE = environment(nX, nY, nZ);

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);
                    
                    

                    % Check if the drone has reached its destination:
                    if (nX == dX && nY == dY && nZ == dZ)
                        % Get the drone to wait for the others to arrive at their respective destinations:
                        drones(d).STATUS = "WAITING";

                        % Incrementing the number of arrived drones:
                        dronesArrived = dronesArrived + 1;
                        continue;
                    end

                    % Performing the same flag check as the M-FA action:
                    if (drones(d).VALUE > minPollution)
                        if (polStates(nX, nY, nZ) > -1)

                            timer = timer + toc;
                            polFlags = Mark_Flag(polFlags, drones(d), iter, timer, totalFlags);
                            tic;
                            
                            polStates = Lock_Area(polStates, drones(d), radius, locked, Ub, Lb);
                            totalFlags = totalFlags + 1;

                            % Wait on other drones for their destinations:
                            drones(d).STATUS = "WAITING";
                            drones(d).MADE_FLAG = true;
                            flagMade = true;
                            dronesArrived = dronesArrived + 1;
                        end
                    end

                    % Skip the rest of the actions for a drone:
                    continue;
                end

                % The drone needs to escape from a flagged area:
                if (role == "ESCAPE")
                    % Getting a position outside the flagged area:
                    drones(d).VELOCITIES = Flag_Escape(dimensions, upMove);

                    % Moving the drone away from the flagged area:
                    drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);

                    % Getting the drone's new position:
                    nPos = drones(d).POSIT;
                    nX = nPos(X); nY = nPos(Y); nZ = nPos(Z);

                    % Assigning the position's value to the drone:
                    drones(d).VALUE = environment(nX, nY, nZ);

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);

                    % Marking another flag, if possible:
                    if (drones(d).VALUE > minPollution)
                        if (locations(nX, nY, nZ) ~= locked)
                            
                            timer = timer + toc;
                            polFlags = Mark_Flag(polFlags, drones(d), iter, timer, totalFlags);
                            tic;
                            
                            polStates = Lock_Area(polStates, drones(d), radius, locked, Ub, Lb);
                            totalFlags = totalFlags + 1;
                        end
                        % Setting the drone to to do normal FA movements:
                    else
                        drones(d).M_ROLE = "GATHER";
                    end
                end
            end
        end

        %% Checking the end of an iteration:
        
        % Recording each drone's position:
        for d = 1:numDrones
            drones(d).POSITIONS(iter+1, 1) = drones(d).POSIT(1);
            drones(d).POSITIONS(iter+1, 2) = drones(d).POSIT(2);
            drones(d).POSITIONS(iter+1, 3) = drones(d).POSIT(3);
            drones(d).POSITIONS(iter+1, 4) = drones(d).VALUE;
        end

        % Checking if all drones have finished their searches:
        if (dronesFinished == numDrones)
            % Immediately stopping further algorithm functions:
            break;
        end

        % Getting all drone to return once most iterations are done:
        if (iter >= iterations * 0.95)
            for d = 1:numDrones
                if (drones(d).STATUS ~= "LANDING" && drones(d).STATUS ~= "FINISHED")
                    % Get the start position of the drone, except altitude:
                    drones(d).DESTINATION(X) = drones(d).START_POSIT(X);
                    drones(d).DESTINATION(Y) = drones(d).START_POSIT(Y);
                    drones(d).DESTINATION(Z) = drones(d).POSIT(Z);

                    drones(d).STATUS = "RETURN";
                    drones(d).ACTIVE = false;
                end
            end

            continue;
        end


        if (noDronesActive == false)
            if (dronesFinished == (numDrones-1))
                % Will set the last active drone to return to base:
                for d = 1:numDrones
                    % Checking if the drone is still searching:
                    if (drones(d).STATUS ~= "LANDING" && drones(d).STATUS ~= "FINISHED")

                        % Get the start position of the drone:
                        drones(d).DESTINATION(X) = drones(d).START_POSIT(X);
                        drones(d).DESTINATION(Y) = drones(d).START_POSIT(Y);
                        drones(d).DESTINATION(Z) = drones(d).START_POSIT(Z);

                        drones(d).STATUS = "RETURN";
                        drones(d).ACTIVE = false;
                        noDronesActive = true;
                        break;
                    end
                end
            end
        end

        % Checking if drones have arrived at their destination,
        % ignoring those who are currently at base:
        if (dronesArrived == (numDrones - dronesInBase))
            % Setting all drones to be in 'GATHER' or 'ESCAPE' action:
            for d = 1:numDrones
                % Checking if the drone is currently searching:
                if (drones(d).ACTIVE == true)
                    % Resetting the drone's destination:
                    drones(d).DESTINATION = zeros(1, dimensions);

                    % Set FA role if flags were found during approaches:
                    if (flagMade == true)
                        % New status depends on whether a flag was found:
                        if (drones(d).MADE_FLAG == true)
                            drones(d).M_ROLE = "ESCAPE";
                        else
                            drones(d).M_ROLE = "GATHER";
                        end
                    else
                        drones(d).M_ROLE = "SELECT";
                    end

                    % Setting each drone back to the 'M-FA' status:
                    drones(d).STATUS = "M-FA";
                    drones(d).MADE_FLAG = false;
                end
            end

            % Resetting the number of drones arriving at destinations:
            dronesArrived = 0;

            % Resetting the status on found flags:
            flagMade = false;
        end

        % Checks to see if the drones should reset every X iterations:
        if (mod(iter, stateCheck) == 0)

            % Resets each of the drones' states if in M-FA:
            for d = 1:numDrones
                % Checks if the drone is in an active state:
                if (drones(d).ACTIVE == true)
                    drones(d).STATUS = "M-FA";
                    drones(d).M_ROLE = "SELECT";
                    drones(d).BEST_VALUE = 0;
                    drones(d).BEST_POSIT = zeros(1, dimensions);
                end
            end

            % Resetting the drones which have arrived:
            dronesArrived = 0;

            % Skip the remaining commands to avoid redundancy:
            continue;
        end
        
        % Decreasing alpha intensity each iteration:
        alpha = alpha * theta;
    end

    %% Outputting the results:
    
    timer = timer + toc;
    
    disp("------------------------------------------------");
    disp("M-FA - Run " + run + ":");
    disp("Total flags found: " + length(polFlags));
    disp("Time taken: " + iter);

    Gather_Results("M-FA", run, iter, timer, dimensions, areas, originPos,... 
        sevPollution, flagRadius, Ub, Lb, polFlags);

end