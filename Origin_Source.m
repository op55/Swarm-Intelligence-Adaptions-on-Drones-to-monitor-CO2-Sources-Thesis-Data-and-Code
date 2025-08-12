classdef Origin_Source
    % Pollution_Source - A pollution source's data.
    %   The information of a pollution source, like its position and value:
    
    properties
        ID;
        POSIT;
        VALUE;
        STEPS_MADE;
        TIME_TAKEN;
        RECHARGES;
        AREA_STEPS;
        AREA_TIME;
        SOURCE_ID;
    end
    
    methods
        function obj = Origin_Source(id, position, value, steps, time, batteries, aSteps, aTime)
            % Constructing a new pollution source:
            obj.ID         = id;
            obj.POSIT      = position;
            obj.VALUE      = value;
            obj.STEPS_MADE = steps;
            obj.TIME_TAKEN = time;
            obj.RECHARGES  = batteries;
            obj.AREA_STEPS = aSteps;
            obj.AREA_TIME  = aTime;
            obj.SOURCE_ID  = inf;
        end

        function obj = Define_Source(obj, source)
            obj.SOURCE_ID = source;
        end
    end
end

