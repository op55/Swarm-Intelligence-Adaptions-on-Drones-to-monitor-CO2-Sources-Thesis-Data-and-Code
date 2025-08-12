classdef Pollution_Flag
    % Pollution_Peak - A pollution peak's data:
    %   All details of an individual pollution peak.
    
    properties
        ID;        
        POSIT;
        VALUE;
        STEPS;
        TIME;
        DRONE;
        RECHARGES;
        SOURCE;
    end
    
    methods
        function obj = Pollution_Flag( id, position, value, step, time, droneID, battery )
            % Constructing a pollution peak:
            obj.ID = id;
            obj.POSIT = position;
            obj.VALUE = value;
            obj.STEPS = step;
            obj.TIME = time;
            obj.DRONE = droneID;
            obj.RECHARGES = (battery-1);
            obj.SOURCE = false;
        end
    end
end

