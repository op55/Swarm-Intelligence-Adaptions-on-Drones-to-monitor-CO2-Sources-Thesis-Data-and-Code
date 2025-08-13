function[states, polSource] = Set_Area(states, polSource, boundaries, bPosit, Lb, Ub, environment, minPollution, sourceID, locked)
% Set_Area - Setting up for the onlookers.
%   Identifying areas for the onlookers, and locking the origin position:
    
    search = boundaries(1); searchZ = boundaries(2);
    source = boundaries(3); sourceZ = boundaries(4);

    X = 1; Y = 2; Z = 3;
    bX = bPosit(X); bY = bPosit(Y); bZ = bPosit(Z);

    % Giving out an area for Onlookers:
    for x = -search:search
        for y = -search:search
            for z = -searchZ:searchZ
                sX = bX + x;
                sY = bY + y;
                sZ = bZ + z;
                sPos = [sX, sY, sZ];
    
                % Checking position is in boundaries:
                if (Boundary_Check(sPos, Lb, Ub) == true)
                    % Checking the pollution is sufficient:
                    if (environment(sX, sY, sZ) > minPollution)
                        % Checking the position is not filled in:
                        if (states(sX, sY, sZ) == 0)
                            states(sX, sY, sZ) = sourceID;
                            polSource = polSource.Add_Suspect([sX, sY, sZ]);
                        end
                    end
                end
            end
        end
    end

    % Locking down the source's origin position:
    for x = -source:source
        for y = -source:source
            for z = -sourceZ:sourceZ
                lX = bX + x;
                lY = bY + y;
                lZ = bZ + z;
                lPos = [lX, lY, lZ];

                % Check position is in boundaries:
                if (Boundary_Check(lPos, Lb, Ub) == true)
                    if (states(lX, lY, lZ) ~= locked)
                        states(lX, lY, lZ) = locked;
                    end
                end
            end
        end
    end

end