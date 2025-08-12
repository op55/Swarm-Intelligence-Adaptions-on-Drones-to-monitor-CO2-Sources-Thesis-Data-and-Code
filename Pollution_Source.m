classdef Pollution_Source
    % Pollution_Source: Details of an individual source:
    %   The details of each pollution source, such as position and ID.
    
    properties
        ID;                     % The ID of a pollution source.
        POSIT;                  % The position of the source.
        VALUE;                  % The value of the source.
        STEPS;                  % Steps taken to find the source.
        TIME;                   % Time taken to find the source.       
        DRONE;                  % The ID of the drone who found the source.
        RECHARGES;              % How many charged batteries it took.
    end
    
    methods
        function obj = Pollution_Source(polID, position, value, timer, steps, droneID, batteries)
            % Pollution_Source -  An instance of a source:
            obj.ID = polID;
            obj.POSIT = position;
            obj.VALUE = value;
            obj.STEPS = steps;
            obj.TIME = timer;            
            obj.DRONE = droneID;
            obj.RECHARGES = (batteries-1);
        end
    end
end

