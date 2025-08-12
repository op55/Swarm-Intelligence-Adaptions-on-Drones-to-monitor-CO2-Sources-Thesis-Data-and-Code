function[newPosition] = Return_To_Base(dimensions, new, old, upMove, Ub, Lb, areas)
% Return_To_Base - Return a drone to its base.
%   Getting a drone to return to base if its battery level is critical:
    
    newPosition = zeros(1, dimensions);
    
    % Going through each dimension:
    for m = 1:dimensions
        
        % Checking the new position value:
        move = new(m) - old(m);
        
        % Limiting the movements made:
        if (move > upMove(m)); move = upMove(m);
        elseif (move < -upMove(m)); move = -upMove(m); end
        
        % Assigning the new dimensional position:
        newPosition(m) = old(m) + move;
        
        % Checking the boundaries:
        if (newPosition(m) > Ub(m)); newPosition(m) = areas(m)-upMove(m);
        elseif (newPosition(m) < Lb(m)); newPosition(m) = Lb(m)+upMove(m); end
    
    end
    
end