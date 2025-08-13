function[] = ZZC_Main(parameters, environment, drones, locations, run)
% ZZC_Main - The main function for Zig-Zag Climber.
%   The functionality of the ZZC algorithm, a linear-based one:

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

    %% ZZC Constants:

    % Numbers to refer to axises:
    X = 1;  Y = 2;  Z = 3;

    STAY_LIMIT = 3;

    % ZZC-exclusive constants:
    DRONE_GROUPS = 2;               % The number of groups available.
    PATH_CORNERS = 31;              % The number of corners for pattern.
    Z_AXIS_START = 27;              % The drone's Z axis takeoff stop.
    SCAN_STEPS = 27;                % Amount of steps for scanning.
    DISABLE_STEPS = 75;             % Number of iterations a drone cannot search.

    %% Standard ZZC Variables:

    % The boundaries for each dimension:
    Ub = [areas(X), areas(Y), areas(Z)];
    Lb = [1, 1, minAltitude];

    % Attributes for the flag areas:
    flagged    = 1;             % Highlights a spot which is flagged.
    radius     = 1;             % How big the radius of a flagged area.
    lockRadius = [25, 25, 2];   % The area size of a source lock.
    flagRadius = [25, 25, 2]; 

    % Attributes for drone groups:
    activeIteration = 125;          % Steps to activate the even drones:

    totalSources = 0;               % For storing the total peaks.
    
    % Attributes for the drone's pathing system:
    horizonLine = 230;              % The X axis distance for the pattern.
    verticalLine = 25;              % The Y axis distance for the pattern. (Set at 25, 15, 5, or 3)
    altitudeLine = 4;               % The Z axis distance for the pattern.

    % Attributes for drone counting:
    dronesFinished = 0;         % How many drones have finished.
    
    %% Pollution Flag Variables:

    % Entries to store for outputs:.
    totalFlags = 0;             % For storing the total flags. 
    
    % Storing each area, with an empty one for a placeholder:
    polSources = Pollution_Source.empty(0, numel(1));

    % Storing each pollution, starting with an empty one for a placeholder:
    polFlags = Pollution_Flag.empty(0, numel(1));

    % Used for reference when marking pollution flags in the area:
    polStates = zeros(areas(X), areas(Y), areas(Z));

    % Used for reference when marking pollution flags:
    flagStates = zeros(areas(X), areas(Y), areas(Z));

    %% Setting up drone path points:

    for d = 1:numDrones
        
        desPoints = zeros(PATH_CORNERS, dimensions);
        cPos = drones(d).POSIT;

        if (mod(d, DRONE_GROUPS) == 0)
            % Even-numbered drones:
            desPoints(1, X)  = cPos(X);                         desPoints(1, Y)  = (cPos(Y) + horizonLine);     desPoints(1, Z)  = Z_AXIS_START;
            desPoints(2, X)  = (cPos(X) - verticalLine);        desPoints(2, Y)  = (cPos(Y) + horizonLine);     desPoints(2, Z)  = Z_AXIS_START;
            desPoints(3, X)  = (cPos(X) - verticalLine);        desPoints(3, Y)  = cPos(Y);                     desPoints(3, Z)  = Z_AXIS_START;
            desPoints(4, X)  = (cPos(X) - (verticalLine*2));    desPoints(4, Y)  = cPos(Y);                     desPoints(4, Z)  = Z_AXIS_START;
            desPoints(5, X)  = (cPos(X) - (verticalLine*2));    desPoints(5, Y)  = (cPos(Y) + horizonLine);     desPoints(5, Z)  = Z_AXIS_START;
            desPoints(6, X)  = (cPos(X) - (verticalLine*3));    desPoints(6, Y)  = (cPos(Y) + horizonLine);     desPoints(6, Z)  = Z_AXIS_START;
            desPoints(7, X)  = (cPos(X) - (verticalLine*3));    desPoints(7, Y)  = cPos(Y);                     desPoints(7, Z)  = Z_AXIS_START;

            desPoints(8, X)  = (cPos(X) - (verticalLine*3));    desPoints(8, Y)  = cPos(Y);                     desPoints(8, Z)  = (Z_AXIS_START - altitudeLine);
            desPoints(9, X)  = (cPos(X) - (verticalLine*3));    desPoints(9, Y)  = (cPos(Y) + horizonLine);     desPoints(9, Z)  = (Z_AXIS_START - altitudeLine);
            desPoints(10, X) = (cPos(X) - (verticalLine*2));    desPoints(10, Y) = (cPos(Y) + horizonLine);     desPoints(10, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(11, X) = (cPos(X) - (verticalLine*2));    desPoints(11, Y) = cPos(Y);                     desPoints(11, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(12, X) = (cPos(X) - verticalLine);        desPoints(12, Y) = cPos(Y);                     desPoints(12, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(13, X) = (cPos(X) - verticalLine);        desPoints(13, Y) = (cPos(Y) + horizonLine);     desPoints(13, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(14, X) = cPos(X);                         desPoints(14, Y) = (cPos(Y) + horizonLine);     desPoints(14, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(15, X) = cPos(X);                         desPoints(15, Y) = cPos(Y);                     desPoints(15, Z) = (Z_AXIS_START - altitudeLine);

            desPoints(16, X) = cPos(X);                         desPoints(16, Y) = cPos(Y);                     desPoints(16, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(17, X) = cPos(X);                         desPoints(17, Y) = (cPos(Y) + horizonLine);     desPoints(17, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(18, X) = (cPos(X) - verticalLine);        desPoints(18, Y) = (cPos(Y) + horizonLine);     desPoints(18, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(19, X) = (cPos(X) - verticalLine);        desPoints(19, Y) = cPos(Y);                     desPoints(19, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(20, X) = (cPos(X) - (verticalLine*2));    desPoints(20, Y) = cPos(Y);                     desPoints(20, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(21, X) = (cPos(X) - (verticalLine*2));    desPoints(21, Y) = (cPos(Y) + horizonLine);     desPoints(21, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(22, X) = (cPos(X) - (verticalLine*3));    desPoints(22, Y) = (cPos(Y) + horizonLine);     desPoints(22, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(23, X) = (cPos(X) - (verticalLine*3));    desPoints(23, Y) = cPos(Y);                     desPoints(23, Z) = (Z_AXIS_START - (altitudeLine*2));

            desPoints(24, X) = (cPos(X) - (verticalLine*3));    desPoints(24, Y) = cPos(Y);                     desPoints(24, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(25, X) = (cPos(X) - (verticalLine*3));    desPoints(25, Y) = (cPos(Y) + horizonLine);     desPoints(25, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(26, X) = (cPos(X) - (verticalLine*2));    desPoints(26, Y) = (cPos(Y) + horizonLine);     desPoints(26, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(27, X) = (cPos(X) - (verticalLine*2));    desPoints(27, Y) = cPos(Y);                     desPoints(27, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(28, X) = (cPos(X) - verticalLine);        desPoints(28, Y) = cPos(Y);                     desPoints(28, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(29, X) = (cPos(X) - verticalLine);        desPoints(29, Y) = (cPos(Y) + horizonLine);     desPoints(29, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(30, X) = cPos(X);                         desPoints(30, Y) = (cPos(Y) + horizonLine);     desPoints(30, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(31, X) = cPos(X);                         desPoints(31, Y) = cPos(Y);                     desPoints(31, Z) = (Z_AXIS_START - (altitudeLine*3));

        else
            % Odd-numbered drones:
            desPoints(1, X)  = cPos(X);                         desPoints(1, Y)  = (cPos(Y) + horizonLine);     desPoints(1, Z)  = Z_AXIS_START;
            desPoints(2, X)  = (cPos(X) + verticalLine);        desPoints(2, Y)  = (cPos(Y) + horizonLine);     desPoints(2, Z)  = Z_AXIS_START;
            desPoints(3, X)  = (cPos(X) + verticalLine);        desPoints(3, Y)  = cPos(Y);                     desPoints(3, Z)  = Z_AXIS_START;
            desPoints(4, X)  = (cPos(X) + (verticalLine*2));    desPoints(4, Y)  = cPos(Y);                     desPoints(4, Z)  = Z_AXIS_START;
            desPoints(5, X)  = (cPos(X) + (verticalLine*2));    desPoints(5, Y)  = (cPos(Y) + horizonLine);     desPoints(5, Z)  = Z_AXIS_START;
            desPoints(6, X)  = (cPos(X) + (verticalLine*3));    desPoints(6, Y)  = (cPos(Y) + horizonLine);     desPoints(6, Z)  = Z_AXIS_START;
            desPoints(7, X)  = (cPos(X) + (verticalLine*3));    desPoints(7, Y)  = cPos(Y);                     desPoints(7, Z)  = Z_AXIS_START;

            desPoints(8, X)  = (cPos(X) + (verticalLine*3));    desPoints(8, Y)  = cPos(Y);                     desPoints(8, Z)  = (Z_AXIS_START - altitudeLine);
            desPoints(9, X)  = (cPos(X) + (verticalLine*3));    desPoints(9, Y)  = (cPos(Y) + horizonLine);     desPoints(9, Z)  = (Z_AXIS_START - altitudeLine);
            desPoints(10, X) = (cPos(X) + (verticalLine*2));    desPoints(10, Y) = (cPos(Y) + horizonLine);     desPoints(10, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(11, X) = (cPos(X) + (verticalLine*2));    desPoints(11, Y) = cPos(Y);                     desPoints(11, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(12, X) = (cPos(X) + verticalLine);        desPoints(12, Y) = cPos(Y);                     desPoints(12, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(13, X) = (cPos(X) + verticalLine);        desPoints(13, Y) = (cPos(Y) + horizonLine);     desPoints(13, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(14, X) = cPos(X);                         desPoints(14, Y) = (cPos(Y) + horizonLine);     desPoints(14, Z) = (Z_AXIS_START - altitudeLine);
            desPoints(15, X) = cPos(X);                         desPoints(15, Y) = cPos(Y);                     desPoints(15, Z) = (Z_AXIS_START - altitudeLine);

            desPoints(16, X) = cPos(X);                         desPoints(16, Y) = cPos(Y);                     desPoints(16, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(17, X) = cPos(X);                         desPoints(17, Y) = (cPos(Y) + horizonLine);     desPoints(17, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(18, X) = (cPos(X) + verticalLine);        desPoints(18, Y) = (cPos(Y) + horizonLine);     desPoints(18, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(19, X) = (cPos(X) + verticalLine);        desPoints(19, Y) = cPos(Y);                     desPoints(19, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(20, X) = (cPos(X) + (verticalLine*2));    desPoints(20, Y) = cPos(Y);                     desPoints(20, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(21, X) = (cPos(X) + (verticalLine*2));    desPoints(21, Y) = (cPos(Y) + horizonLine);     desPoints(21, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(22, X) = (cPos(X) + (verticalLine*3));    desPoints(22, Y) = (cPos(Y) + horizonLine);     desPoints(22, Z) = (Z_AXIS_START - (altitudeLine*2));
            desPoints(23, X) = (cPos(X) + (verticalLine*3));    desPoints(23, Y) = cPos(Y);                     desPoints(23, Z) = (Z_AXIS_START - (altitudeLine*2));

            desPoints(24, X) = (cPos(X) + (verticalLine*3));    desPoints(24, Y) = cPos(Y);                     desPoints(24, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(25, X) = (cPos(X) + (verticalLine*3));    desPoints(25, Y) = (cPos(Y) + horizonLine);     desPoints(25, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(26, X) = (cPos(X) + (verticalLine*2));    desPoints(26, Y) = (cPos(Y) + horizonLine);     desPoints(26, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(27, X) = (cPos(X) + (verticalLine*2));    desPoints(27, Y) = cPos(Y);                     desPoints(27, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(28, X) = (cPos(X) + verticalLine);        desPoints(28, Y) = cPos(Y);                     desPoints(28, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(29, X) = (cPos(X) + verticalLine);        desPoints(29, Y) = (cPos(Y) + horizonLine);     desPoints(29, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(30, X) = cPos(X);                         desPoints(30, Y) = (cPos(Y) + horizonLine);     desPoints(30, Z) = (Z_AXIS_START - (altitudeLine*3));
            desPoints(31, X) = cPos(X);                         desPoints(31, Y) = cPos(Y);                     desPoints(31, Z) = (Z_AXIS_START - (altitudeLine*3));
            
            % Getting these drones to launch as the algorithm starts:
            drones(d).ZZC_USE = true;
        end
        
        drones(d).PATH_POINTS = desPoints;
    end

    %% Setting up the other position-based arrays:

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

    
    %% Starting ZZC

    % Starting the timer:
    timer = 0;  tic;

    for iter = 1:iterations

        % Update environment at each time step
        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );

        % Iterating through the drones
        for d = 1:numDrones

            if (drones(d).FINISHED == true); continue; end

            if (drones(d).ZZC_USE == true)
                
                % Decreasing the drone's battery life, if it is in use:
                if (drones(d).STATUS ~= "REPLACING")
                    drones(d) = Battery_Deplete(drones(d), dimensions, decreaseRatio, lowBattery);
                end
                
                % Determining counts if a drone goes back to base or is dead:
                if (drones(d).ACTIVE == false)
                    if (drones(d).NEEDS_RETURN == false)
                        drones(d).NEEDS_RETURN = true;
                    end                
                end

                if (drones(d).FINISHED == true); dronesFinished = (dronesFinished + 1); end

                status = drones(d).STATUS;
                customRole = drones(d).ZZC_ROLE;
                sPos = drones(d).START_POSIT;

                %% Checking if the drone has connection to its base:

                if (status == "TAKEOFF")
                    
                    % Moving one space higher than the current Z axis:
                    drones(d).POSIT(Z) = (drones(d).POSIT(Z) + 1);
                    nX = drones(d).POSIT(X);
                    nY = drones(d).POSIT(Y);
                    nZ = drones(d).POSIT(Z);
                    drones(d).VALUE = environment(nX, nY, nZ);                    

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);

                    % Commence ABC role once desired altitude is reached:
                    if (nZ == Z_AXIS_START)
                        drones(d).STATUS = "CUSTOM";
                    end
                    
                    continue;
                end

                if (status == "RETURN")
                    
                    oPos = drones(d).POSIT;
                    dPos = drones(d).DESTINATION;
                    
                    % Making movements for each dimension:
                    drones(d).POSIT = Return_To_Base(dimensions, dPos, oPos, upMove, Ub, Lb, areas);
                    nX = drones(d).POSIT(X);
                    nY = drones(d).POSIT(Y);
                    nZ = drones(d).POSIT(Z);
                    drones(d).VALUE = environment(nX, nY, nZ);  

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);

                    % Checking if the drone's has reaching its starting point:
                    if (nX == sPos(X) && nY == sPos(Y))
                        drones(d).STATUS = "LANDING";
                    end
                    
                    continue;
                end

                if (status == "LANDING")                    
                    % Moving the drone so its Z axis is one position lower:
                    drones(d).POSIT(Z) = (drones(d).POSIT(Z) - 1);
                    nX = drones(d).POSIT(X);
                    nY = drones(d).POSIT(Y);
                    nZ = drones(d).POSIT(Z);
                    drones(d).VALUE = environment(nX, nY, nZ);  

                    % Updating the new location on the map:
                    [drones(d), locations] = Update_Map(drones(d), locations);

                    % Checking the drone is back to its starting position:
                    if (nZ == sPos(Z))
                        
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
                        drones(d).NEEDS_RETURN = false;

                        if (drones(d).ZZC_ROLE == "SCAN")
                            drones(d).DESTINATION = drones(d).LAST_POSIT;
                            drones(d).START_FOUND = true;                            
                        else
                            drones(d).START_FOUND = false;
                        end
                        
                        drones(d).ZZC_ROLE = "SELECT";

                    end

                    continue;
                end
                
                %% Select a corner point to start on:

                if (customRole == "SELECT")
                    
                    % Checking if a drone has found a corner to start on:
                    if (drones(d).START_FOUND == false)
                    
                        % Selecting a random corner up the 2/3 of the way:
                        cornerStart = randi([1, 18]);
                        
                        % Setting a corner as a destination:
                        dX = drones(d).PATH_POINTS(cornerStart, X);
                        dY = drones(d).PATH_POINTS(cornerStart, Y);
                        dZ = drones(d).PATH_POINTS(cornerStart, Z);
                        
                        dPos = [dX, dY, dZ];
                        drones(d).DESTINATION = dPos;

                        % Setting up the next corner for the drone:
                        drones(d).CURRENT_POINT = (cornerStart+1);
                        
                        % Drone has found a corner:
                        drones(d).START_FOUND = true;
                    
                    % The drone needs to approach to its corner to start:
                    else
                        
                        % Shortening the position variables:
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

                        % Updating the new location on the map:
                        [drones(d), locations] = Update_Map(drones(d), locations);

                        % Checking if a path point has been reached:
                        if (nX == dPos(X) && nY == dPos(Y) && nZ == dPos(Z))
                            %
                            if (drones(d).NEW_SEARCH == true)
                                drones(d).ZZC_ROLE = "SEARCH";
                            else
                                drones(d).ZZC_ROLE = "SCAN";
                                drones(d).NEW_SEARCH = true;
                            end
                        end

                    end

                    continue;
                end


                %% Commiting to the normal search:
                
                if (status == "CUSTOM")

                    if (customRole == "SEARCH")

                        oPos = drones(d).POSIT;
                        dPoint = drones(d).CURRENT_POINT;
                        disable = drones(d).DISABLE_STATE;

                        % Getting the drone's current path point:
                        dX = drones(d).PATH_POINTS(dPoint, X);
                        dY = drones(d).PATH_POINTS(dPoint, Y);
                        dZ = drones(d).PATH_POINTS(dPoint, Z);
                        dPos = [dX, dY, dZ];

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

                        % Checking if a path point has been reached:
                        if (nX == dPos(X) && nY == dPos(Y) && nZ == dPos(Z))
                            drones(d).CURRENT_POINT = dPoint + 1;

                            % Checking if the drone has reached the end of its search path:
                            if (drones(d).CURRENT_POINT > PATH_CORNERS)
                                drones(d).STATUS = "LANDING";
                                continue;
                            end
                        end

                        % Check if the drone is able to search this iteration:
                        if (disable > 0)
                            drones(d).DISABLE_STATE = (disable - 1);
                            continue;
                        end

                        % Checking if the pollution value is sufficient:
                        if (drones(d).VALUE > minPollution)
                            % Checking if the area has been locked:
                            if (polStates(nX, nY, nZ) == 0)
                                drones(d).SCAN_START = drones(d).POSIT;
                                drones(d).ZZC_ROLE = "SCAN";
                            end
                        end

                        continue;
                    end

                    % The drone is doing a scan for better positions:
                    if (customRole == "SCAN")
                        
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

                        if (ZZC_Check(bPosit, Lb, Ub) == true)
                            
                            if (drones(d).STAY_COUNTER == STAY_LIMIT)

                                if (bValue >= sevPollution && polStates(nX, nY, nZ) == 0)

                                    % Setting up variables in shorter naming conventions:
                                    bestPosit = drones(d).POSIT;
                                    bestValue = drones(d).VALUE;
                                    droneID = drones(d).ID;
                                    droneBatteries = drones(d).BATTERY_PACKS;

                                    % Incrementing the total number of sources:
                                    totalSources = totalSources + 1;

                                    % Showing the time it took to find the peak:
                                    timer = timer + toc;

                                    % Setting up the source position:
                                    polSources(totalSources) = Pollution_Source(totalSources, bestPosit, bestValue, timer, iter, droneID, droneBatteries);

                                    % Starting the timer again:
                                    tic;

                                    bX = bestPosit(X);
                                    bY = bestPosit(Y);
                                    bZ = bestPosit(Z);

                                    % Lock down the pollution area:
                                    for x = -lockRadius(X):lockRadius(X)
                                        for y = -lockRadius(Y):lockRadius(Y)
                                            for z = -lockRadius(Z):lockRadius(Z)
                                                lX = bX + x;
                                                lY = bY + y;
                                                lZ = bZ + z;
                                                lPos = [lX, lY, lZ];

                                                % Check position is in boundaries:
                                                if (ZZC_Check(lPos, Lb, Ub) == true)
                                                    if (polStates(lX, lY, lZ) == 0)
                                                        polStates(lX, lY, lZ) = drones(d).ID;
                                                    end
                                                end
                                            end
                                        end
                                    end

                                    % Getting the drone to go back to its first scanning position:
                                    drones(d).ZZC_ROLE = "GET_BACK";

                                end

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
                                drones(d).ZZC_ROLE = "GET_BACK";
                            end
                        else
                            drones(d).ZZC_ROLE = "GET_BACK";
                        end

                        continue;
                    end

                    if (customRole == "GET_BACK")

                        oPos = drones(d).POSIT;

                        % Getting the drone's starting scan coordinate:
                        sX = drones(d).SCAN_START(X);
                        sY = drones(d).SCAN_START(Y);
                        sZ = drones(d).SCAN_START(Z);
                        sPos = [sX, sY, sZ];

                        % Getting the velocities for the drone:
                        drones(d).VELOCITIES = Get_Destination(dimensions, sPos, drones(d).POSIT, upMove, Z);

                        % Moving a drone with the 'Move_Drone' function:
                        drones(d) = Move_Drone(dimensions, drones(d), Ub, Lb);

                        % Getting the drone's new position:
                        nPos = drones(d).POSIT;
                        nX = nPos(X); nY = nPos(Y); nZ = nPos(Z);
                        
                        % Assigning the position's value to the drone:
                        drones(d).VALUE = environment(nX, nY, nZ);
                        
                        % Changing the drone's location on the map:
                        locations(oPos(X), oPos(Y), oPos(Z)) = 0;
                        locations(nX, nY, nZ) = drones(d).ID;

                        % Checking if the drone is back to where it started:
                        if (nPos(X) == sPos(X) && nPos(Y) == sPos(Y) && nPos(Z) == sPos(Z))
                            % Getting the drone to a 'disabled' search;
                            drones(d).ZZC_ROLE = "SEARCH";
                            drones(d).DISABLE_STATE = DISABLE_STEPS;
                            drones(d).STAY_COUNTER = 0;
                        end

                    end

                end

                
                
            end

        end

        %% Checking the end of an iteration:
        
        % Storing each drone's positions to their paths:
        for d = 1:numDrones
            drones(d).POSITIONS(iter+1, X) = drones(d).POSIT(X);
            drones(d).POSITIONS(iter+1, Y) = drones(d).POSIT(Y);
            drones(d).POSITIONS(iter+1, Z) = drones(d).POSIT(Z);
            drones(d).POSITIONS(iter+1, 4) = drones(d).VALUE;
        end

        % Finish searching once all drones are finished:
        if (dronesFinished == numDrones); break; end

        % Getting all drone to return once mpost iterations are done:
        if (iter >= iterations * 0.95)
            for d = 1:numDrones
                if (drones(d).STATUS ~= "RETURN" && drones(d).STATUS ~= "LANDING" && drones(d).STATUS ~= "FINISHED")
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
        
        % After a number of iterations, activate more drones:
        if (iter == activeIteration)
            for d = 1:numDrones
                if (drones(d).ZZC_USE == false)
                    drones(d).ZZC_USE = true;
                end
            end
        end

        

    end

    %% Outputting the results:
    
    timer = timer + toc;
    
    disp("------------------------------------------------");
    disp("ZZC - Run " + run + ":");
    disp("Total flags found: " + length(polFlags));
    disp("Time taken: " + iter);
    
    % % Plotting Drone 1's Search Path:
    % x = drones(1).POSITIONS(:,1);
    % y = drones(1).POSITIONS(:,2);
    % z = drones(1).POSITIONS(:,3);
    % 
    % x(all(x == 0, 2), :) = [];
    % y(all(y == 0, 2), :) = [];
    % z(all(z == 0, 2), :) = [];
    % 
    % % Plot the 3D path
    % figure; hold on;
    % plot3(x, y, z, '-o', 'LineWidth', 1);  % '-o' adds markers at each point
    % grid on;
    % clim([400, 1000]);
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % title('Scenario 1 - ZZC Search Path Example');
    % view(2);
    % 
    % [xOutline, yOutline, zOutline] = meshgrid([1, areas(1)], [1, areas(2)], [1, areas(3)]);
    % 
    % % Plot corners in the X-Y planes at Z=1 and Z=matrixSize(3)
    % scatter3(xOutline(:), yOutline(:), ones(numel(xOutline), 1), 1, 'k', 'filled');
    % scatter3(xOutline(:), yOutline(:), areas(3) * ones(numel(xOutline), 1), 1, 'k', 'filled');
    % 
    % % Plot corners in the X-Z planes at Y=1 and Y=matrixSize(2)
    % scatter3(xOutline(:), ones(numel(xOutline), 1), zOutline(:), 1, 'k', 'filled');
    % scatter3(xOutline(:), areas(2) * ones(numel(xOutline), 1), zOutline(:), 1, 'k', 'filled');
    % 
    % % Plot corners in the Y-Z planes at X=1 and X=matrixSize(1)
    % scatter3(ones(numel(yOutline), 1), yOutline(:), zOutline(:), 1, 'k', 'filled');
    % scatter3(areas(1) * ones(numel(yOutline), 1), yOutline(:), zOutline(:), 1, 'k', 'filled');
    % 
    % hold off;

    Gather_Results("ZZC", run, iter, timer, dimensions, areas, originPos,... 
        sevPollution, flagRadius, Ub, Lb, polFlags, polSources);

end