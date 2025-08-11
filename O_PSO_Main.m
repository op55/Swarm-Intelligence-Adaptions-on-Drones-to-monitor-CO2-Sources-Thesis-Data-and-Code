function[] = O_PSO_Main(parameters, environment, run)

    %% Getting the parameters into their own variables:
    
    GENERAL            = parameters{1};
    iterations         = GENERAL(1); 
    numDrones          = GENERAL(3);
    
    SEARCHES           = parameters{2};
    minAltitude        = SEARCHES(1);    
    
    SCENARIOS          = parameters{3};
    decayRate          = SCENARIOS(3);
    windInfluence      = SCENARIOS(4);
    
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

    %% Commencing O-PSO:

    % Dimensions of the environment
    [xDim, yDim, zDim] = deal(areas(1), areas(2), areas(3));
    
    % O-PSO coefficients
    w  = 0.7;    % inertia weight
    c1 = 0.5;    % cognitive (personal best)
    c2 = 1.5;    % social (global best)

    % Initialize particle positions and velocities
    positions = [randi([1, xDim], numDrones, 1), ...
                 randi([1, yDim], numDrones, 1), ...
                 randi([minAltitude, zDim], numDrones, 1)];
    velocities = zeros(numDrones, 3);

    % Initialize personal and global bests
    pBestPos = positions;
    pBestVal = zeros(numDrones, 1);

    gBestVal = -inf;
    gBestPos = [0, 0, 0];
    gBestIter = 0;
    gBestTime = 0;
    gBestHistory = zeros(iterations, 1);
    
    timer = 0; tic;

    % Main O-PSO loop
    for iter = 1:iterations

        % Update environment at each time step
        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );

        for i = 1:numDrones
            pos = positions(i, :);
            value = environment(pos(1), pos(2), pos(3));

            % Update personal best
            if value > pBestVal(i)
                pBestVal(i) = value;
                pBestPos(i, :) = pos;
            end

            % Update global best
            if value > gBestVal
                gBestVal = value;
                gBestPos = pos;
                gBestIter = iter;
                
                timer = timer + toc;
                gBestTime = timer;
                tic;                
            end
        end

        gBestHistory(iter) = gBestVal;

        % Update velocities and positions
        for i = 1:numDrones
            r1 = rand(1, 3);
            r2 = rand(1, 3);
            velocities(i, :) = w * velocities(i, :) + ...
                               c1 * r1 .* (pBestPos(i, :) - positions(i, :)) + ...
                               c2 * r2 .* (gBestPos - positions(i, :));
            newPos = round(positions(i, :) + velocities(i, :));

            % Clamp to grid boundaries
            newPos = min(max(newPos, [1, 1, minAltitude]), [xDim, yDim, zDim]);

            positions(i, :) = newPos;
        end
    end
    
    timer = timer + toc;

    Output("O-PSO", run, gBestVal, gBestPos, gBestIter, gBestTime, iter, timer);
end