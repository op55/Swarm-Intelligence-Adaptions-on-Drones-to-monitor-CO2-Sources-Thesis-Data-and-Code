function[valid] = Boundary_Check(position, Lb, Ub)
% Boundary_Check - Checks on a drone's new position.
%   Ensures each drone's dimensional position is within the search area:
    
    valid = false;
    validParts = 0;
    
    % Going through each dimension to check if they are in boundary:
    for i = 1:length(position)
        if (position(i) >= Lb(i) && position(i) <= Ub(i))
            validParts = (validParts + 1);
        else
            break;
        end
    end
    
    % Checks if all dimenional positions are in boundary:
    if (validParts == length(position))
        valid = true;
    end

end

