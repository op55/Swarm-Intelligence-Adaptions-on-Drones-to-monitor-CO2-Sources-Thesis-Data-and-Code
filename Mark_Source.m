function[trueSources, flagChecks] = Mark_Source(trueSources, flagChecks, polFlags, currentFlag,...
                                              sPos, sVal, sStep, sTime, sCharges, radius, Lb, Ub)
% Mark_Source - Adding a new source.
%   Creating a new source, storing its ID, position, value, etc.
    
    X = 1; Y = 2; Z = 3;
    
    % Setting up the source ID and it total number of flags:
    newEntry = length(trueSources) + 1;
    totalFlags = length(polFlags);
    
    % Initialising some of the source's data entries:
    areaSteps = inf;
    areaTime = inf;

    % Setting the area for the source:
    for x = -radius(X):radius(X)
        for y = -radius(Y):radius(Y)
            for z = -radius(Z):radius(Z)
                cX = sPos(X) + x;
                cY = sPos(Y) + y;
                cZ = sPos(Z) + z;
                cPos = [cX, cY, cZ];
                
                % Checking if position is in boundaries:
                if (Boundary_Check(cPos, Lb, Ub) == true)
                    if (flagChecks(cX, cY, cZ) == 0)
                        % Giving the position the source's ID:
                        flagChecks(cX, cY, cZ) = newEntry;
                    end
                end                
            end
        end
    end

    % Collecting the flags within the source area:
    for flag = currentFlag:totalFlags

        fPosit = polFlags(flag).POSIT;
        fSteps = polFlags(flag).STEPS;
        fTimer = polFlags(flag).TIME;

        if (flagChecks(fPosit(X), fPosit(Y), fPosit(Z)) == newEntry)
            % Getting the lowest timer and steps within the area:            
            if (fSteps < areaSteps); areaSteps = fSteps; end
            if (fTimer < areaTime); areaTime = fTimer; end
        end

    end

    trueSources(newEntry) = Origin_Source(newEntry, sPos, sVal, sStep, sTime, sCharges, areaSteps, areaTime);

end