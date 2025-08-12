function[] = Gather_Results(algorithm, run, iter, timer, dimensions, areas,...
    originPos, sevPollution, flagRadius, Ub, Lb, polFlags, polSources)
% Gather_Results - Getting the data from the RD Algorithms.
%   Collecting information about the pollution flags, and extracting data:
    
    X = 1; Y = 2; Z = 3;

    %% Creating a plot:

    % Create a figure for the output:
    figure; hold on;

    % Plot the flags on the map:
    for flag = 1:length(polFlags)
        fPos = polFlags(flag).POSIT;
        scatter3(fPos(X), fPos(Y), fPos(Z), 10, polFlags(flag).VALUE, 'filled');
    end

    % Define the colormap to use for the plot:
    colorbar;
    numColors = 256;
    cmap = zeros(numColors, 3);

    % Transitioning between colours based on pollution value, from blue, green, yellow, and then red:
    cmap(1:floor(numColors/3), :) = [linspace(0,0,floor(numColors/3))', linspace(0,1,floor(numColors/3))', linspace(1,0,floor(numColors/3))'];
    cmap(floor(numColors/3)+1:2*floor(numColors/3), :) = [linspace(0,1,floor(numColors/3))', ones(floor(numColors/3),1), linspace(0,0,floor(numColors/3))'];
    cmap(2*floor(numColors/3)+1:end, :) = [ones(numColors-floor(2*numColors/3),1), linspace(1,0,numColors-floor(2*numColors/3))', linspace(0,0,numColors-floor(2*numColors/3))'];

    % Applying the custom colours on to the plot:
    colormap(cmap);    

    xlabel('X-Axis');
    ylabel('Y-Axis');
    zlabel('Z-Axis');
    clim([400, 1000]);
    title('Pollution Flags - ZZC');

    [xOutline, yOutline, zOutline] = meshgrid([1, areas(1)], [1, areas(2)], [1, areas(3)]);

    % Plot corners in the X-Y planes at Z=1 and Z=matrixSize(3)
    scatter3(xOutline(:), yOutline(:), ones(numel(xOutline), 1), 10, 'k', 'filled');
    scatter3(xOutline(:), yOutline(:), areas(3) * ones(numel(xOutline), 1), 10, 'k', 'filled');

    % Plot corners in the X-Z planes at Y=1 and Y=matrixSize(2)
    scatter3(xOutline(:), ones(numel(xOutline), 1), zOutline(:), 10, 'k', 'filled');
    scatter3(xOutline(:), areas(2) * ones(numel(xOutline), 1), zOutline(:), 10, 'k', 'filled');

    % Plot corners in the Y-Z planes at X=1 and X=matrixSize(1)
    scatter3(ones(numel(yOutline), 1), yOutline(:), zOutline(:), 10, 'k', 'filled');
    scatter3(areas(1) * ones(numel(yOutline), 1), yOutline(:), zOutline(:), 10, 'k', 'filled');

    % Adjust the view for top-down visualization:
    view(0, 90);    axis tight;     grid on;
    hold off;

    %% Identifying the expected origins of the sources:
    
    % Getting the sources of each pollution area:
    trueSources = Origin_Source.empty(0, numel(1));
    flagChecks = zeros(areas(X), areas(Y), areas(Z));
    
    % Ordering the flags based on their pollution value:
    pollutionValues = [polFlags.VALUE];
    [~, sortedIndices] = sort(pollutionValues, 'descend');
    polFlags = polFlags(sortedIndices);

    % Ordering the sources based on their values:
    pollutionSources = [polSources.VALUE];
    [~, sortedIndices] = sort(pollutionSources, 'descend');
    polSources = polSources(sortedIndices);
    
    for flag = 1:length(polSources)       
        % Break the loop if pollution values are less than the severity:
        if (polSources(flag).VALUE < sevPollution); break; end

        pPos     = polSources(flag).POSIT;
        pVal     = polSources(flag).VALUE;
        pSteps   = polSources(flag).STEPS;
        pTimer   = polSources(flag).TIME;
        pCharges = polSources(flag).RECHARGES;

        % Checks if the area is not in an origin area:
        if (flagChecks(pPos(X), pPos(Y), pPos(Z)) == 0)

            % For telling if an origin position is nearby:
            relation = false;
            relatedOrigin = 0;

            boundX = 5; boundY = 5; boundZ = 1;

            % Checks the surrounding area around the position:
            for x = (pPos(X)-boundX):(pPos(X)+boundX)
                for y = (pPos(Y)-boundY):(pPos(Y)+boundY)
                    for z = (pPos(Z)-boundZ):(pPos(Z)+boundZ)

                        cX = pPos(X) + x;
                        cY = pPos(Y) + y;
                        cZ = pPos(Z) + z;
                        cPos = [cX, cY, cZ];

                        % Checking if position is in boundaries:
                        if (Boundary_Check(cPos, Lb, Ub) == true)
                            if (flagChecks(x, y, z) ~= 0)
                                relatedOrigin = flagChecks(x, y, z);
                                relation = true;
                            end
                        end


                    end
                end
            end

            % Checks if the flag has got any origin positions nearby:
            if (relation == false)

                % Marking a new source from the area:
                [trueSources, flagChecks] = Mark_Source(...
                    trueSources, flagChecks, polFlags, flag, pPos,... 
                    pVal, pSteps, pTimer, pCharges, flagRadius, Lb, Ub...
                );

            else

                % Extending the lock radius of the related origin:
                for x = (pPos(X)-boundX):(pPos(X)+boundX)
                    for y = (pPos(Y)-boundY):(pPos(Y)+boundY)
                        for z = (pPos(Z)-boundZ):(pPos(Z)+boundZ)
                            cX = pPos(X) + x;
                            cY = pPos(Y) + y;
                            cZ = pPos(Z) + z;
                            cPos = [cX, cY, cZ];

                            % Checking if position is in boundaries:
                            if (Boundary_Check(cPos, Lb, Ub) == true)
                                if (flagChecks(x, y, z) == 0)
                                    flagChecks(x, y, z) = relatedOrigin;
                                end
                            end
                        end
                    end
                end

            end
        end
    end
    
    % Setting up some arrays for reference:
    sources = length(trueSources);
    origins = size(originPos, 1);
    originConfs = zeros(1, origins);

    % Identifying which sources are in their intended locations:
    for source = 1:sources
        
        % Referencing the shortest distance and source ID:
        shortestDistance = inf;
        sourceID = inf;
        
        % Iterating through the true original positions:
        for origin = 1:origins           
            
            % Skip if the origin position is already confirmed:
            if (originConfs(origin) == true); continue; end
            
            distance = 0;

            for dim = 1:dimensions
                % Getting the origin and source positions:
                oPos = originPos(origin, dim);
                sPos = trueSources(source).POSIT(dim);
                
                % Measuring the distance of each dimension:
                if (sPos > oPos) 
                    distance = distance + (sPos - oPos);
                else
                    distance = distance + (oPos - sPos);
                end
            end

            % Check if the new total distance is the shortest:
            if (shortestDistance > distance)
                shortestDistance = distance;
                sourceID = origin;
            end
        end  

        % Check if an origin position has been confirmed:
        if (sourceID ~= inf)
            trueSources(source) = trueSources(source).Define_Source(sourceID);
            originConfs(sourceID) = true;
        end

        % Skip if the discovered positions are less than intended:
        if (source == sources); break; end
    end

    % Ordering the pollution sources based on PPs, ascending:
    sourceIDs = [trueSources.SOURCE_ID];
    [~, sortedSources] = sort(sourceIDs, 'ascend');
    trueSources = trueSources(sortedSources);

    %% Placing the results into a text file:
    
    % Remembering the current discovered source to use:
    currentSource = 1;

    % Outputting each pollution source into a text file:
    for PP = 1:origins
        
        valid = false;

        % Checking if there are any entries for discovered sources:
        if (isempty(sourceIDs) == false)
            % Check if the given PP has been found (based on set ordering):
            if (ismember(PP, sourceIDs)); valid = true; end
        end

        if (valid == true)
            dPos       = trueSources(currentSource).POSIT;
            dPosit     = ("[" + dPos(X) + ", " + dPos(Y) + ", " + dPos(Z) + "]");
            dValue     = sprintf('%0.2f', trueSources(currentSource).VALUE);
            dSteps     = sprintf('%0.1f', trueSources(currentSource).STEPS_MADE);
            dTime      = sprintf('%0.2f', trueSources(currentSource).TIME_TAKEN);
            dBatteries = sprintf('%0.1f', trueSources(currentSource).RECHARGES);
            aSteps     = sprintf('%0.1f', trueSources(currentSource).AREA_STEPS);
            aTime      = sprintf('%0.2f', trueSources(currentSource).AREA_TIME);
            info       = [dPosit, dValue, dSteps, dTime, dBatteries, aSteps, aTime];
            
            % Remembering the next discovered source to use:
            currentSource = currentSource + 1;
        else
            info = ["N/A", 0, 0, 0, 0, 0, 0];
        end

        fileName = algorithm + "_PP" + PP + ".txt";
        Write_Results(fileName, info);
    end

    % Outputting general information about an algorithm run:
    fileName     = algorithm + "_Overall.txt";
    runNo        = sprintf('%0.1f', run);
    foundSources = sprintf('%0.1f', length(trueSources));
    flags        = sprintf('%0.1f', length(polFlags));   
    runIter      = sprintf('%0.1f', iter);
    timer        = sprintf('%0.2f', timer);
    
    Write_Overalls(fileName, runNo, foundSources, flags, runIter, timer);

end