function [] = O_ABC_Main(parameters, environment, run)

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


    %% Performing O-ABC:
    
    % The variables to use for outputs:
    bestValue = -inf;      % start with very low value
    bestPos = [0, 0, 0];   % placeholder
    bestIter = 0;          % time step not yet know
    bestTime = 0;
    
    % The size of the environment's dimensions:
    [xDim, yDim, zDim] = size(environment);
    
    % Step 1: Initialize random positions
    xVals = randi([1, xDim], numDrones, 1);
    yVals = randi([1, yDim], numDrones, 1);
    zVals = randi([minAltitude, zDim], numDrones, 1);
    pos = [xVals, yVals, zVals];

    fitness = arrayfun(@(i) environment(pos(i,1), pos(i,2), pos(i,3)), 1:numDrones)';

    timer = 0; tic;

    for iter = 1:iterations

        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );
        
        % Recalculate the fitness value:
        fitness = arrayfun(@(i) environment(pos(i,1), pos(i,2), pos(i,3)), 1:numDrones)';

        % Step 2: Employed Bee Phase (explore neighbors)
        for i = 1:numDrones
            newPos = pos(i,:) + randi([-1,1],1,3); % small move
            newPos = min(max(newPos, 1), [xDim, yDim, zDim]);
            newFit = environment(newPos(1), newPos(2), newPos(3));
            if newFit > fitness(i)
                pos(i,:) = newPos;
                fitness(i) = newFit;
            end
        end

        % Step 3: Onlooker Bee Phase
        probs = fitness / sum(fitness);
        for i = 1:numDrones
            if rand < probs(i)
                j = randi(numDrones);
                newPos = pos(j,:) + randi([-1,1],1,3);
                newPos = min(max(newPos, 1), [xDim, yDim, zDim]);
                newFit = environment(newPos(1), newPos(2), newPos(3));
                if newFit > fitness(i)
                    pos(i,:) = newPos;
                    fitness(i) = newFit;
                end
            end
        end

        % Step 4: Scout Bee Phase
        for i = 1:numDrones
            if rand < 0.1  % abandonment probability
                xVals = randi([1, xDim], 1, 1);
                yVals = randi([1, yDim], 1, 1);
                zVals = randi([minAltitude, zDim], 1, 1);
                pos(i,:) = [xVals, yVals, zVals];
                fitness(i) = environment(pos(i,1), pos(i,2), pos(i,3));
            end
        end

        % Get current best from this iteration
        [maxFit, bestIdx] = max(fitness);
        currBestPos = pos(bestIdx, :);

        % Compare to all-time best
        if maxFit > bestValue
            bestValue = maxFit;
            bestPos = currBestPos;
            bestIter = iter;

            timer = timer + toc;
            bestTime = timer;
            tic;

        end
    end
    
    timer = timer + toc;

    % Step 5: Return best solution
    [~, bestIdx] = max(fitness);
    bestSolution = pos(bestIdx, :);

    Output("O-ABC", run, bestValue, bestPos, bestIter, bestTime, iter, timer);
end