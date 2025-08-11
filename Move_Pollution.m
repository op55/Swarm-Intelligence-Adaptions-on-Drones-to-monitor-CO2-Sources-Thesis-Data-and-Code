function[environment, newOrigin] = Move_Pollution(...
    iter, oldEnvironment, areas, smokes, scales, decayRate, windInfluence,... 
    originPos, ppmMax, pStartTimes, pStopTimes, pMinMove, pSpreads, pVelocities,...
    initialWinds, finalWinds, transitionStart, transitionDuration...
)
% Move_Pollution - Generating Dynamic Pollution over time.
%   Detailed explanation goes here

    [x, y, z] = deal(areas(1), areas(2), areas(3)); 
    [density, sigma] = deal(smokes(1), smokes(2));
    [scalingFactor, zScale, seedNum] = deal(scales(1), scales(2), scales(3));      
    

    % Determining the wind direction based on iteration:
    if iter < transitionStart
        currentWind = initialWinds;
    elseif iter >= transitionStart && iter <= transitionStart + transitionDuration
        t = (iter - transitionStart) / transitionDuration;  % Goes from 0 to 1
        currentWind = (1 - t) * initialWinds + t * finalWinds;  % smooth linear interpolation
    else
        currentWind = finalWinds;
    end
    
    environment = oldEnvironment * (1 - decayRate);
    newOrigin = originPos;
    
    seed = iter * seedNum;
    rng(seed);
    
    for o = 1:size(originPos, 1)
    
        if iter < pStartTimes(o) || iter > pStopTimes(o)
            continue;
        end
    
        newOrigin(o, :) = originPos(o, :) + pVelocities(o, :);
        plumeAge = iter - pStartTimes(o) + 1;
    
        px = min(pSpreads(o,1) + plumeAge * pSpreads(o,3), pSpreads(o,2));
        py = min(pSpreads(o,4) + plumeAge * pSpreads(o,6), pSpreads(o,5));
        pz = min(pSpreads(o,7) + plumeAge * pSpreads(o,9), pSpreads(o,8));
    
        newX = randn(density, 1);
        newY = randn(density, 1);
        newZ = randn(density, 1);
    
        newX = newX .* px;
        newY = newY .* py;
        newZ = newZ .* pz;
    
        distances = sqrt(newX.^2 + newY.^2 + newZ.^2);
        scaleFactor = exp(-distances.^2 / (2 * sigma^2));
    
        growthFactor = min(plumeAge / 10, 1);
    
    
        newX = newX + currentWind(1) * distances * growthFactor * windInfluence;
        newY = newY + currentWind(2) * distances * growthFactor * windInfluence;
        newZ = newZ + currentWind(3) * distances * zScale * growthFactor * windInfluence;
    
        newX = newX * scalingFactor;
        newY = newY * scalingFactor;
        newZ = newZ * scalingFactor;
    
        scaleFactor = scaleFactor * ppmMax(o);
    
        originPosition = newOrigin(o, :);
        newX = newX + originPosition(1);
        newY = newY + originPosition(2);
        newZ = newZ + originPosition(3);
    
        minZ = max(originPosition(3), pMinMove(o));
        newZ = max(newZ, minZ);
    
        xIdx = round(newX);    yIdx = round(newY);    zIdx = round(newZ);
        xIdx = max(min(xIdx, x), 1);
        yIdx = max(min(yIdx, y), 1);
        zIdx = max(min(zIdx, z), 1);
    
        for p = 1:length(xIdx)
            environment(xIdx(p), yIdx(p), zIdx(p)) = scaleFactor(p);
        end
    end
    
    environment = min(environment, 1000);
    
    % % Filling out the remaining spaces for the area with randomness:
    % for cX = 1:x
    %     for cY = 1:y
    %         for cZ = minAltitude:z
    %             if (environment(cX, cY, cZ) <= 50)
    %                 environment(cX, cY, cZ) = (rand(1) * 50);
    %             end
    %         end
    %     end
    % end

    rng('shuffle');
    
end