function[] = M_ABC_Main(parameters, environment, drones, locations, run)
    
    X = 1;  Y = 2;  Z= 3;

    %% Getting the parameters into their own variables:
    
    GENERAL            = parameters{1};
    iterations         = GENERAL(1);
    dimensions         = GENERAL(2);
    numDrones          = GENERAL(3);
    
    SEARCHES           = parameters{2};
    minAltitude        = SEARCHES(1);
    
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
    
    %% M-ABC Parameters:

    ROLE_SHIFT = 2;                 % Determine roles for drone at start.
    DISABLED_TIME = 150;            % Time an employer/scout is disabled.

    % Employer Bee Constants:
    SCAN_STEPS = 27;
    SLEEP_TIME = floor(iterations*0.250); % 1000

    % Onlooker Bee Constants:
    ATTEMPTS = 10;

    % Determines how many drones are trying to takeoff:
    droneTakeoffs = numDrones;
    onlookerTakeoffs = 0;
    
    % The search boundaries for the Onlookers:
    searchArea  = 40;           % The size for onlookers to search in.
    searchAreaZ = 2;            % The size for the Z axis in search areas.
    sourceArea  = 4;            % The size of the source to lock down.
    sourceAreaZ = 1;            % The size for the Z axis in source areas.
    
    % The source parameters:
    totalSources = 0;           % For storing the total peaks.
    finishedSources = 0;        % Remembering the total discovered sources.

    % Attributes for drone counting:
    dronesFinished = 0;         % How many drones have finished.
    scoutsFinished = 0;         % How many scouts are in use.
    
    
    %% Standard ABC Variables:
    
    % Assigning each dimension into a variable:
    areaX = areas(1);   areaY = areas(2);   areaZ = areas(3);

    % The boundaries for each dimension:
    Ub = [areaX, areaY, areaZ];
    Lb = [1, 1, minAltitude];

    % Attributes for the flag areas:
    radius     = 1;             % How big the radius of a flagged area.
    sourceLock = -1;            % Indicates a disabled coordinate for the Onlookers.
    flagged    = 1;             % Highlights a spot which is flagged.
    flagRadius = [25, 25, 2];

    
    STAY_LIMIT = 3;
    

    %% Pollution Flag Variables:

    % Entries to store for outputs:.
    totalFlags = 0;             % For storing the total flags.   

    % Storing each area, with an empty one for a placeholder:
    polSources = Pollution_Source.empty(0, numel(1));

    % Storing each pollution, starting with an empty one for a placeholder:
    polFlags = Pollution_Flag.empty(0, numel(1));

    % Used for reference when marking pollution areas:
    polStates = zeros(areaX, areaY, areaZ);
    
    % Used for reference when marking pollution flags:
    flagStates = zeros(areaX, areaY, areaZ);

    %% Setting up the roles for the drones:

    for d = 1:numDrones
        if (mod(d, ROLE_SHIFT) == 0)
            drones(d).BEE_ROLE = "ONLOOKER";
            drones(d).STATUS = "WAITING";
            drones(d).LAUNCH = false;
            onlookerTakeoffs = onlookerTakeoffs + 1;
        else
            drones(d).BEE_ROLE = "SCOUT"; 
            drones(d).LAUNCH = true;
            scoutsFinished = scoutsFinished + 1;
        end
    end

    %% Setting up the selection numbers for some roles:

    newPol = zeros(SCAN_STEPS, dimensions);

    newPol(1,X)  = 0;            newPol(1,Y)  = 0;            newPol(1,Z)  = upMove(Z);
    newPol(2,X)  = upMove(X);    newPol(2,Y)  = 0;            newPol(2,Z)  = upMove(Z);
    newPol(3,X)  = upMove(X);    newPol(3,Y)  = upMove(Y);    newPol(3,Z)  = upMove(Z);
    newPol(4,X)  = 0;            newPol(4,Y)  = upMove(Y);    newPol(4,Z)  = upMove(Z);
    newPol(5,X)  = -upMove(X);   newPol(5,Y)  = upMove(Y);    newPol(5,Z)  = upMove(Z);
    newPol(6,X)  = -upMove(X);   newPol(6,Y)  = 0;            newPol(6,Z)  = upMove(Z);
    newPol(7,X)  = -upMove(X);   newPol(7,Y)  = -upMove(Y);   newPol(7,Z)  = upMove(Z);
    newPol(8,X)  = 0;            newPol(8,Y)  = -upMove(Y);   newPol(8,Z)  = upMove(Z);
    newPol(9,X)  = upMove(X);    newPol(9,Y)  = -upMove(Y);   newPol(9,Z)  = upMove(Z);
    newPol(10,X) = upMove(X);    newPol(10,Y) = 0;            newPol(10,Z) = 0;
    newPol(11,X) = upMove(X);    newPol(11,Y) = upMove(Y);    newPol(11,Z) = 0;
    newPol(12,X) = 0;            newPol(12,Y) = upMove(Y);    newPol(12,Z) = 0;
    newPol(13,X) = -upMove(X);   newPol(13,Y) = upMove(Y);    newPol(13,Z) = 0;
    newPol(14,X) = -upMove(X);   newPol(14,Y) = 0;            newPol(14,Z) = 0;
    newPol(15,X) = -upMove(X);   newPol(15,Y) = -upMove(Y);   newPol(15,Z) = 0;
    newPol(16,X) = 0;            newPol(16,Y) = -upMove(Y);   newPol(16,Z) = 0;
    newPol(17,X) = upMove(X);    newPol(17,Y) = -upMove(Y);   newPol(17,Z) = 0;
    newPol(18,X) = upMove(X);    newPol(18,Y) = 0;            newPol(18,Z) = -upMove(Z);
    newPol(19,X) = upMove(X);    newPol(19,Y) = upMove(Y);    newPol(19,Z) = -upMove(Z);
    newPol(20,X) = 0;            newPol(20,Y) = upMove(Y);    newPol(20,Z) = -upMove(Z);
    newPol(21,X) = -upMove(X);   newPol(21,Y) = upMove(Y);    newPol(21,Z) = -upMove(Z);
    newPol(22,X) = -upMove(X);   newPol(22,Y) = 0;            newPol(22,Z) = -upMove(Z);
    newPol(23,X) = -upMove(X);   newPol(23,Y) = -upMove(Y);   newPol(23,Z) = -upMove(Z);
    newPol(24,X) = 0;            newPol(24,Y) = -upMove(Y);   newPol(24,Z) = -upMove(Z);
    newPol(25,X) = upMove(X);    newPol(25,Y) = -upMove(Y);   newPol(25,Z) = -upMove(Z);
    newPol(26,X) = 0;            newPol(26,Y) = 0;            newPol(26,Z) = -upMove(Z);
    newPol(27,X) = 0;            newPol(27,Y) = 0;            newPol(27,Z) = 0;

    %% Starting M-ABC:

    % Starting the timer:
    timer = 0;  tic;

    for iter = 1:iterations
        
        if (iter == (iterations - 1))
            disp("Error has happened.");
        end

        % Update environment at each time step
        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );

        % Iterating through the number of drones for battery depletion:
        for d = 1:numDrones

            if (drones(d).STATUS == "WAITING"); continue; end

            % Checks if the current drone has finished:
            if (drones(d).FINISHED == true)
                % Checking if the drone has not performed this body:
                if (drones(d).BEE_ROLE ~= "EXHAUSTED")
                    dronesFinished = (dronesFinished + 1);

                    % Checking if the drone is an employer or scout:
                    if(drones(d).BEE_ROLE == "EMPLOYER" || drones(d).BEE_ROLE == "SCOUT")
                        scoutsFinished = scoutsFinished - 1;

                        % Checking if all employers/scouts are finished:
                        if (scoutsFinished == 0)
                            % Get all onlookers to stop their roles and return:
                            for m = 1:numDrones
                                if (drones(m).BEE_ROLE == "ONLOOKER")
                                    if (drones(m).STATUS == "WAITING")
                                        drones(m).FINISHED = true;
                                        drones(m).BEE_ROLE = "EXHAUSTED";
                                        dronesFinished = (dronesFinished + 1);
                                    elseif (drones(m).ACTIVE == true)
                                        % Will not interrupt onlookers who are search for spots:
                                        if (drones(m).ONLOOKER_STANDBY == false)
                                            drones(m).STATUS = "RETURN";
                                            drones(m).DESTINATION = drones(m).START_POSIT;
                                            drones(m).ACTIVE = false;
                                        end
                                    end
                                end
                            end
                        end
                    end

                    % Prevents further deactivations from this drone:
                    drones(d).BEE_ROLE = "EXHAUSTED";
                end

                continue;
            end

            % Decreasing the drone's battery life, depending on movement:
            if (drones(d).STATUS ~= "REPLACING" && drones(d).LAUNCH == true)
                drones(d) = Battery_Deplete(drones(d), dimensions, decreaseRatio, lowBattery);
            end


            %% Performing base-related action, when possible:

            status = drones(d).STATUS;
            sPos = drones(d).START_POSIT;

            % The drone is taking off from its base:
            if (status == "TAKEOFF")

                % Checks if the drone cannot launch currently:
                if (drones(d).LAUNCH == false); continue; end

                % Letting the algorithm know the drone performed:
                drones(d).PERFORMED = true;

                % Moving one space higher than the current Z axis:
                drones(d).POSIT(Z) = (drones(d).POSIT(Z) + 1);
                nPos = drones(d).POSIT;

                % Assigning the position's value to the drone:
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Commence ABC role once desired altitude is reached:
                if (drones(d).POSIT(Z) == minAltitude)

                    if (drones(d).BEE_ROLE == "EMPLOYER")
                        drones(d).STATUS = "BACK_TO_ROLE";
                    elseif (drones(d).BEE_ROLE == "ONLOOKER")
                        drones(d).STATUS = "ABC";
                        onlookerTakeoffs = onlookerTakeoffs - 1;
                    elseif (drones(d).BEE_ROLE == "SCOUT")
                        drones(d).STATUS = "ABC";
                    end

                    % Letting everyone know the drone has taken off:
                    if (droneTakeoffs > 0); droneTakeoffs = (droneTakeoffs - 1); end
                end

                % Skip the rest of the actions for a drone:
                continue;
            end

            % The drone is returning back above its base:
            if (status == "RETURN")
                % Letting the algorithm know the drone performed:
                drones(d).PERFORMED = true;

                % Shortening some variables:
                dPos = drones(d).START_POSIT;
                oPos = drones(d).POSIT;

                % Making movements for each dimension:
                drones(d).POSIT = Return_To_Base(dimensions, dPos, oPos, upMove, Ub, Lb, areas);
                nPos = drones(d).POSIT;

                % Assigning the position's value to the drone:
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Checking if the drone's has reaching its starting point:
                if (nPos(X) == dPos(X) && nPos(Y) == dPos(Y))
                    drones(d).STATUS = "LANDING";
                end

                % Skip the rest of the actions for a drone:
                continue;
            end

            % The is drone is landing on its base:
            if (status == "LANDING")
                % Letting the algorithm know the drone performed:
                drones(d).PERFORMED = true;

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
                        continue;
                    end

                    % Shortening the name for batteries a drone has used:
                    batteries = drones(d).BATTERY_PACKS;

                    % Checking if the drone is an onlooker:
                    if (drones(d).BEE_ROLE == "ONLOOKER")

                        % Checking if all scouts have finished:
                        if (scoutsFinished == 0)
                            % Declaring the drone as finished:
                            drones(d).FINISHED = true;
                            continue;
                        end

                        % Check if battery is at a high-enough level:
                        if (drones(d).BATTERY > (lowBattery*3))
                            % Keep onlooker active for takeoffs:
                            drones(d).LAUNCH = false;
                            drones(d).STATUS = "WAITING";
                            onlookerTakeoffs = onlookerTakeoffs + 1;
                            continue;
                        end

                    end

                    % Checking if the drone has any batteries left:
                    if (batteries < totalBatteries)
                        % Getting the drone to replace its battery:
                        drones(d).BATTERY_PACKS = (batteries + 1);
                        drones(d).REPLACE_PHASE = replaceTime;
                        drones(d).STATUS = "REPLACING";
                    else
                        % Declaring the drone as finished:
                        drones(d).FINISHED = true;
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
                    drones(d).STATUS = "TAKEOFF";
                    drones(d).ACTIVE = true;
                    drones(d).BATTERY = 100.0;

                    if (drones(d).BEE_ROLE == "EMPLOYER")
                        drones(d).DESTINATION = drones(d).LAST_POSIT;
                    elseif (drones(d).BEE_ROLE == "ONLOOKER")
                        drones(d).LAUNCH = false;
                        drones(d).ONLOOKER_STANDBY = true;
                        drones(d).DESTINATION = drones(d).LAST_POSIT;
                        drones(d).STATUS = "WAITING";
                        onlookerTakeoffs = (onlookerTakeoffs + 1);
                    elseif (drones(d).BEE_ROLE == "SCOUT")
                        drones(d).DEST_SEARCH = true;
                    end
                end

                % Skip the rest of the actions for a drone:
                continue;
            end

            if (status == "BACK_TO_ROLE")
                % Shortening some variables:
                lPos = drones(d).LAST_POSIT;

                % Getting the velocities for the drone:
                drones(d).VELOCITIES = Get_Destination(dimensions, dPos, drones(d).POSIT, upMove, Z);

                % Moving a drone with the 'Move_Drone' function:
                drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);
                
                % Assigning the position's value to the drone:
                nPos = drones(d).POSIT;                
                drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                % Updating the new location on the map:
                [drones(d), locations] = Update_Map(drones(d), locations);

                % Checking if the drone's has reaching its starting point:
                if (nPos(X) == lPos(X) && nPos(Y) == lPos(Y) && nPos(Z) == lPos(Z))
                    drones(d).STATUS = "ABC";
                end

                % Skip the rest of the actions for a drone:
                continue;
            end

        end

        %% The ABC algorithm's actions:


        % Skip the ABC roles if no drones are in the search space:
        if (droneTakeoffs < numDrones)

            %% The Employers:

            % Iterating through the employer drones:
            for d = 1:numDrones

                % Checks the drone's condition:
                if (drones(d).LAUNCH == false); continue; end
                if (drones(d).FINISHED == true); continue; end
                if (drones(d).PERFORMED == true); continue; end

                % Checking if the drone can do ABC searching:
                if (drones(d).STATUS == "ABC")
                    % Checking if the drone is an employer:
                    if (drones(d).BEE_ROLE == "EMPLOYER")

                        drones(d).PERFORMED = true;

                        pPosit = drones(d).POSIT;
                        pValue = drones(d).VALUE;
                        pStay = drones(d).STAY_COUNTER;

                        [bPosit, bValue] = Hill_Climb(dimensions, pPosit, pValue, SCAN_STEPS, newPol, Lb, Ub, environment);
                        
                        drones(d).POSIT = bPosit;
                        drones(d).VALUE = bValue;
                        nX = bPosit(X); nY = bPosit(Y); nZ = bPosit(Z);

                        [drones(d), locations] = Update_Map(drones(d), locations);

                        if (bPosit == pPosit)
                            drones(d).STAY_COUNTER = (pStay + 1);
                        else
                            drones(d).STAY_COUNTER = 0;
                        end

                        
                        if (Boundary_Check(bPosit, Lb, Ub) == true)
                            
                            if (drones(d).STAY_COUNTER == STAY_LIMIT)

                                if (bValue >= sevPollution && polStates(nX, nY, nZ) == 0)

                                    % Setting up variables in shorter naming conventions:
                                    bestPosit = drones(d).POSIT;
                                    bestValue = drones(d).VALUE;
                                    droneID = drones(d).ID;
                                    droneBatteries = drones(d).BATTERY_PACKS;
                                    searchBoundaries = [searchArea, searchAreaZ, sourceArea, sourceAreaZ];

                                    % Incrementing the total number of sources:
                                    totalSources = totalSources + 1;

                                    % Getting the source ID for the new area:
                                    drones(d).CURRENT_SOURCE = totalSources;

                                    % Showing the time it took to find the peak:
                                    timer = timer + toc;

                                    % Setting up the source position:
                                    polSources(totalSources) = Pollution_Source(totalSources, bestPosit, bestValue, timer, iter, droneID, droneBatteries, SLEEP_TIME);

                                    % Starting the timer again:
                                    tic;

                                    % Discovering blank spots for onlookers:
                                    [polStates, polSources(totalSources)] = Set_Area(polStates, polSources(totalSources), searchBoundaries, bPosit, Lb, Ub, environment, minPollution, totalSources, sourceLock);

                                end

                                drones(d).BEE_ROLE = "SCOUT";

                            elseif (bValue >= minPollution)
                                
                                if (flagStates(nX, nY, nZ) ~= flagged)
                                    
                                    % Increasing the total number of flags:
                                    totalFlags = totalFlags + 1;

                                    % Showing the time it took to find the peak:
                                    timer = timer + toc;

                                    % Setting up a flag and locking its area:
                                    [flagStates, polFlags] = Mark_Flag(flagStates, polFlags, drones(d), radius, Lb, Ub, flagged, totalFlags, iter, timer);
                                    
                                    % Starting the timer again:
                                    tic;
                                
                                end

                            else
                                drones(d).BEE_ROLE = "SCOUT";
                            end

                        else                           
                            drones(d).BEE_ROLE = "SCOUT";
                        end
                        
                        % Checking if the Employer has become a Scout:
                        if (drones(d).BEE_ROLE == "SCOUT")
                            drones(d).DISABLED_STEPS = DISABLED_TIME;
                            drones(d).STAY_COUNTER = 0;
                        end
                        
                    end
                end
            end

            %% Checking any pollution source discoveries:

            % Checking if a source has just been discovered:
            if (finishedSources < totalSources)

                % Checking if any onlooker have yet to launch:
                if (onlookerTakeoffs > 0)
                    % Enabling the onlookers, if currently disabled:
                    for m = 1:numDrones
                        if (drones(m).BEE_ROLE == "ONLOOKER")
                            if (drones(m).LAUNCH == false)
                                drones(m).LAUNCH = true;
                                drones(m).STATUS = "TAKEOFF";
                            elseif (drones(m).STATUS == "RETURN" && drones(m).ACTIVE == true)
                                % Setting the onlooker back to searching:
                                drones(m).STATUS = "ABC";
                            end
                        end
                    end
                end

                % Iterating through all the discovered sources:
                for s = 1:totalSources
                    if (polSources(s).IS_ACTIVE == false); continue; end

                    % Getting the source's current counter:
                    activeCounter = polSources(s).ACTIVE_COUNTER;

                    if (activeCounter > 0)
                        % Only decrement if the timer is active:
                        if (polSources(s).TIMER_ACTIVE == true)
                            polSources(s).ACTIVE_COUNTER = (activeCounter - 1);
                        end
                    else
                        % Declaring the source area as inactive:
                        polSources(source).IS_ACTIVE = false;

                        % Locking down the source area:
                        polStates = Lock_Area(polStates, searchArea, searchAreaZ, polSources(source).POSIT, Lb, Ub, sourceLock);

                        % Incrementing the number of finished sources:
                        finishedSources = finishedSources + 1;
                    end
                end
            end

            % Checking if all discovered sources are inactive:
            if (finishedSources == totalSources)
                % Getting the onlookers to return to base:
                for m = 1:numDrones
                    if (drones(m).BEE_ROLE == "ONLOOKER")
                        if (drones(m).STATUS == "ABC")
                            drones(m).STATUS = "RETURN";
                            drones(m).ONLOOKER_STANDBY = true;
                            onlookerTakeoffs = onlookerTakeoffs + 1;
                        end
                    end
                end
            end

            %% The Onlookers:

            % Iterating through the onlookers:
            for d = 1:numDrones

                % Checks the drone's condition:
                if (drones(d).LAUNCH == false); continue; end
                if (drones(d).FINISHED == true); continue; end
                if (drones(d).PERFORMED == true); continue; end

                if (drones(d).STATUS == "ABC")
                    % Checking if the drone is an onlooker:
                    if (drones(d).BEE_ROLE == "ONLOOKER")

                        % Letting the algorithm know the drone performed:
                        drones(d).PERFORMED = true;

                        % Checks if the onlooker is on standby:
                        if (drones(d).ONLOOKER_STANDBY == false)

                            source = drones(d).CURRENT_SOURCE;
                            posit = drones(d).POSIT;

                            % Checking if the area has been deactivated:
                            if (polSources(source).IS_ACTIVE == false)
                                drones(d).ONLOOKER_STANDBY = true;
                                drones(d).CURRENT_SOURCE = 0;
                                continue;
                            end

                            % Doing a number of attempts on possible spots:
                            for a = 1:ATTEMPTS
                                % Picking one of 25 possible directions:
                                nPosID = randi([2, SCAN_STEPS-2]);

                                nX = posit(X) + newPol(nPosID, X);
                                nY = posit(Y) + newPol(nPosID, Y);
                                nZ = posit(Z) + newPol(nPosID, Z);
                                nPos = [nX, nY, nZ];

                                if (Boundary_Check(nPos, Lb, Ub) == true)
                                    if (environment(nX, nY, nZ) > minPollution)
                                        if (flagStates(nX, nY, nZ) ~= flagged)
                                            % Moving the drone and updating the map:
                                            drones(d).POSIT = [nX, nY, nZ];
                                            % Assigning the position's value to the drone:
                                            drones(d).VALUE = environment(nX, nY, nZ);
                                            [drones(d), locations] = Update_Map(drones(d), locations);

                                            % Proceeding to add a new flag:
                                            timer = timer + toc;
                                            totalFlags = totalFlags + 1;
                                            [flagStates, polFlags] = Mark_Flag(flagStates, polFlags, drones(d), radius, Lb, Ub, flagged, totalFlags, iter, timer);

                                            % Restarting the timer and breaking the loop:
                                            tic; break;
                                        end
                                    end

                                end
                            end

                            % Checking if the drone did not mark a flag:
                            if (a == ATTEMPTS)
                                drones(d).ONLOOKER_STANDBY = true;
                                drones(d).CURRENT_SOURCE = 0;
                            end

                            % The onlooker is on standby:
                        else

                            % Checking if the drone needs a destination:
                            if (drones(d).DEST_SEARCH == true)

                                % Determines if a destination is found:
                                foundSource = false;

                                % Going through the available sources:
                                for source = 1:totalSources
                                    % Check if source is active:
                                    if (polSources(source).IS_ACTIVE == true)
                                        % Try a number of attempts for spots:
                                        for a = 1:ATTEMPTS
                                            % Getting the drone to pick a suspected spot:
                                            destination = randi(polSources(source).SPOT_COUNT - 1);

                                            dX = polSources(source).SUSPECT_SPOTS{destination}(X);
                                            dY = polSources(source).SUSPECT_SPOTS{destination}(Y);
                                            dZ = polSources(source).SUSPECT_SPOTS{destination}(Z);
                                            dPos = [dX, dY, dZ];

                                            % Ensuring the drone has a unique destination:
                                            if (locations(dX, dY, dZ) == 0)
                                                drones(d).CURRENT_SOURCE = source;
                                                drones(d).DESTINATION = dPos;
                                                drones(d).DEST_SEARCH = false;

                                                % Activate the source timer if it's not already:
                                                if (polSources(source).TIMER_ACTIVE == false)
                                                    polSources(source).TIMER_ACTIVE = true;
                                                end

                                                % Drone has found a spot to go to:
                                                foundSource = true;
                                                break;
                                            end
                                        end
                                    end
                                    if (foundSource == true); break; end
                                end


                                % The drone can move to a destination:
                            else
                                % Shortening some variables:
                                source = drones(d).CURRENT_SOURCE;
                                dPos = drones(d).DESTINATION;

                                % Getting the velocities for the drone:
                                drones(d).VELOCITIES = Get_Destination(dimensions, dPos, drones(d).POSIT, upMove, Z);

                                % Moving a drone with the 'Move_Drone' function:
                                drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);

                                % Getting the drone's new position:
                                nPos = drones(d).POSIT;
                                nX = nPos(X); nY = nPos(Y); nZ = nPos(Z);

                                % Assigning the position's value to the drone:
                                drones(d).VALUE = environment(nX, nY, nZ);

                                [drones(d), locations] = Update_Map(drones(d), locations);

                                % Check if destination has been reached:
                                if (nX == dPos(X) && nY == dPos(Y) && nZ == dPos(Z))
                                    drones(d).ONLOOKER_STANDBY = false;
                                    drones(d).DESTINATION = zeros(1, dimensions);
                                    drones(d).DEST_SEARCH = true;
                                end

                                % Checking if the source is no longer active:
                                if (polSources(source).IS_ACTIVE == false)
                                    drones(d).DESTINATION = zeros(1, dimensions);
                                    drones(d).DEST_SEARCH = true;
                                end
                            end
                        end
                    end
                end
            end


            %% The Scouts:

            % Iterating through the scouts:
            for d = 1:numDrones

                % Checks the drone's condition:
                if (drones(d).LAUNCH == false); continue; end
                if (drones(d).FINISHED == true); continue; end
                if (drones(d).PERFORMED == true); continue; end

                if (drones(d).STATUS == "ABC")
                    % Checking if the drone is a scout:
                    if (drones(d).BEE_ROLE == "SCOUT")

                        % Letting the algorithm know the drone performed:
                        drones(d).PERFORMED = true;

                        % Check if the drone needs to search for a destination:
                        if (drones(d).DEST_SEARCH == true)

                            % Setting up each dimensional position:
                            dX = randi([Lb(X), Ub(X)]);
                            dY = randi([Lb(Y), Ub(Y)]);
                            dZ = randi([Lb(Z), Ub(Z)]);

                            % Giving the drone its new destination:
                            drones(d).DESTINATION = [dX, dY, dZ];
                            drones(d).DEST_SEARCH = false;
                            continue;

                            % The drone already has a destination:
                        else

                            % Shortening some variables:
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
                            drones(d).VALUE = environment(nPos(X), nPos(Y), nPos(Z));

                            [drones(d), locations] = Update_Map(drones(d), locations);

                            % Checking if the drone is at its destination:
                            if (nX == dX && nY == dY && nZ == dZ)
                                drones(d).DEST_SEARCH = true;
                            end

                            % Ignore polluted area if drone's disabled:
                            if (drones(d).DISABLED_STEPS > 0)
                                drones(d).DISABLED_STEPS = drones(d).DISABLED_STEPS - 1;
                                continue;
                            end

                            % Checking if drone is in a polluted area:
                            if (drones(d).VALUE >= minPollution)
                                % Check the area has not been seen before:
                                if (polStates(nX, nY, nZ) == 0)
                                    drones(d).BEE_ROLE = "EMPLOYER";
                                    drones(d).DEST_SEARCH = true;
                                    continue;
                                end
                            end
                        end
                    end
                end
            end

        end

        %% Checking the end of an iteraion:

        % Resetting every drone performance for the next iteration:
        for d = 1:numDrones
            drones(d).PERFORMED = false;
        end

        % Storing each drone's positions to their paths:
        for d = 1:numDrones
            drones(d).POSITIONS(iter+1, X) = drones(d).POSIT(X);
            drones(d).POSITIONS(iter+1, Y) = drones(d).POSIT(Y);
            drones(d).POSITIONS(iter+1, Z) = drones(d).POSIT(Z);
            drones(d).POSITIONS(iter+1, 4) = drones(d).VALUE;
        end

        % Getting all drone to return once mpost iterations are done:
        if (iter >= iterations * 0.95)
            for d = 1:numDrones
                if (drones(d).STATUS ~= "LANDING" && drones(d).STATUS ~= "FINISHED" && drones(d).STATUS ~= "WAITING")
                    % Get the start position of the drone, except altitude:
                    drones(d).DESTINATION(X) = drones(d).START_POSIT(X);
                    drones(d).DESTINATION(Y) = drones(d).START_POSIT(Y);
                    drones(d).DESTINATION(Z) = drones(d).POSIT(Z);

                    drones(d).STATUS = "RETURN";
                    drones(d).ACTIVE = false;

                    if (drones(d).BEE_ROLE == "ONLOOKER")
                        drones(d).ONLOOKER_STANDBY = true;
                        onlookerTakeoffs = onlookerTakeoffs + 1;
                    end
                end
            end
        end

        % Break the loop if all drones have finished:
        if (dronesFinished == numDrones); break; end

    end

    %% Outputting the results:
    
    timer = timer + toc;
    
    disp("------------------------------------------------");
    disp("M-ABC - Run " + run + ":");
    disp("Total flags found: " + length(polFlags));
    disp("Time taken: " + iter);

    Gather_Results("M-ABC", run, iter, timer, dimensions, areas, originPos,... 
        sevPollution, flagRadius, Ub, Lb, polFlags, polSources);

end