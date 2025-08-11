function [] = O_FA_Main(parameters, environment, run)

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
    
    
    %% Performing O-FA:
    
    % The variables for storing 'best' data:
    bestValue = -inf;      % start with very low value
    bestPos = [0, 0, 0];   % placeholder
    bestIter = 0;          % time step not yet know
    bestTime = 0;
    
    [xDim, yDim, zDim] = size(environment);
    alpha = 1;   % randomness factor
    theta = 0.97;


    % Step 1: Initialize random fireflies
    xVals = randi([1, xDim], numDrones, 1);
    yVals = randi([1, yDim], numDrones, 1);
    zVals = randi([minAltitude, zDim], numDrones, 1);
    pos = [xVals, yVals, zVals];

    brightness = arrayfun(@(i) environment(pos(i,1), pos(i,2), pos(i,3)), 1:numDrones)';

    timer = 0; tic;

    for iter = 1:iterations
        
        [environment, originPos] = Move_Pollution( ...
            iter, environment, areas, smokes, scales, decayRate, windInfluence,...
            originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
            initialWinds, finalWinds, transitionStart, transitionDuration...
        );
    
        brightness = arrayfun(@(i) environment(pos(i,1), pos(i,2), pos(i,3)), 1:numDrones)';

        for i = 1:numDrones
            for j = 1:numDrones
                if brightness(j) > brightness(i)
                    r = (pos(j,:) - pos(i,:));
                    boost = floor((r * alpha) / 2);
                    move = (r - boost);

                    newPos = round(pos(i,:) + move);
                    newPos = min(max(newPos, 1), [xDim, yDim, zDim]);
                    newBright = environment(newPos(1), newPos(2), newPos(3));
                    if newBright > brightness(i)
                        pos(i,:) = newPos;
                        brightness(i) = newBright;
                    end
                end
            end
        end

        % Get current best from this iteration
        [maxFit, bestIdx] = max(brightness);
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

        alpha = alpha * theta;  % gradually reduce randomness

    end

    timer = timer + toc;

    % Return brightest firefly
    [~, bestIdx] = max(brightness);
    bestSolution = pos(bestIdx, :);

    Output("O-FA", run, bestValue, bestPos, bestIter, bestTime, iter, timer);
end