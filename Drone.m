classdef Drone
    % Drone: The drones used for the simulation.
    %   Contains information for each drone while 
    %   they are searching for air pollution.
    
    properties        
        % General Drone Variables:
        ID;             % Drone ID.
        STATUS;         % Drone's status, like 'takeoff' or 'searching'.
        
        BATTERY;        % Tracks the life of a drone while functioning.
        BATTERY_PACKS;  % Tracks how many battery packs a drone has left.
        NEEDS_RETURN;   % Used to increment the returning drones once.
        REPLACE_PHASE;  % Tracks the state of a battery getting replaced/
        
        ACTIVE;         % Determines if a drone can search.
        FINISHED;       % Determines if a drone should be checked.
        MADE_FLAG;      % Used to know if a drone has flagged an area.

        VALUE;          % Drone's current pollution value.
        BEST_VALUE;     % Drone's best pollution value.

        POSIT;          % Drone's current position.
        PREV_POSIT;     % Referencing a drone's previous position.
        BEST_POSIT;     % Drone's best position (highest pollution value).
        START_POSIT;    % Drone's starting position.
        LAST_POSIT;     % Drone's position before facing battery issues.

        POSITIONS;      % All the drone drone's position through searching.
        
        TOTAL_DISTANCE; % The total distance a drone has travelled:

        DESTINATION;    % Drone's destination position (when desired).

        % Variables used for RD-PSO and RD-FA:
        RD_ROLE;    % Used to determine what action a PSO drone does.
        VELOCITIES;     % Used to determine a drone's movement speed.

        % Variables used for RD-ABC and ZZC's Scanning Functionality:
        STAY_COUNTER;               % The iteration a drones stays in a spot.
        THRESHOLD_FOUND;            % Tells a drone has found severe pollution.
        
        % Variables used for RD-ABC:
        LAUNCH;                     % Used to allow the drone to take off.        
        BEE_ROLE;                   % Used to set a role for a drone.
        PERFORMED;                  % Tells when a drone has done a role.
        DEST_SEARCH;                % When drones must find a destination.
        
        CURRENT_SOURCE;             % ID of the area a drone is in.        

        DISABLED_STEPS;             % How many steps a scout cannot find.

        ONLOOKER_STANDBY;           % Tells if onlooker is on standby;
        SOURCES_SEARCHED;           % What sources have been searched.

        % Variables used for ZZC:
        ZZC_ROLE;                % The role for the Custom algorithm.
        ZZC_USE;                 % Determine if a drone can search.
        
        PATH_POINTS;                % The corners of a drone's pattern.
        CURRENT_POINT;              % The point the drone needs to head to.
        SCAN_START;                 % The starting position of a scan.
        DISABLE_STATE;              % The state where a drone cannot search.
        START_FOUND;                % Sets a corner to start on.
        NEW_SEARCH;                 % Tells if a drone is starting from new.
    end
    
    methods
        function obj = Drone(maxIter, total, values, posits, velocs)
            % Initialising each drone entry.

            % Ensuring there are actual entries:
            if (nargin ~= 0)
                drones = size(total, 1);
                obj(drones) = obj;

                % Making an array of drone objects:
                for i = 1:total
                    
                    obj(i).ID = i;
                    obj(i).STATUS = "TAKEOFF";
                    
                    obj(i).BATTERY = 100.00;
                    obj(i).BATTERY_PACKS = 1;
                    obj(i).NEEDS_RETURN = false;
                    obj(i).REPLACE_PHASE = 0;
                    
                    obj(i).ACTIVE = true;
                    obj(i).FINISHED = false;
                    obj(i).MADE_FLAG = false;

                    obj(i).VALUE = values(i);
                    obj(i).BEST_VALUE = values(i);

                    obj(i).POSIT = [posits(i, 1), posits(i, 2), posits(i, 3)];
                    obj(i).PREV_POSIT = [posits(i, 1), posits(i, 2), posits(i, 3)];
                    obj(i).BEST_POSIT = [posits(i, 1), posits(i, 2), posits(i, 3)];
                    obj(i).START_POSIT = [posits(i, 1), posits(i, 2), posits(i, 3)];
                    obj(i).LAST_POSIT = [posits(i, 1), posits(i, 2), posits(i, 3)];

                    obj(i).POSITIONS = zeros(maxIter+1, 4);
                    obj(i).POSITIONS(1, 1) = obj(i).POSIT(1);
                    obj(i).POSITIONS(1, 2) = obj(i).POSIT(2);
                    obj(i).POSITIONS(1, 3) = obj(i).POSIT(3);
                    obj(i).POSITIONS(1, 4) = obj(i).VALUE;
                    
                    obj(i).TOTAL_DISTANCE = 0;

                    obj(i).DESTINATION = zeros(1, 3);

                    % RD-PSO/FA Variables:
                    obj(i).RD_ROLE = "SELECT";
                    obj(i).VELOCITIES(1) = velocs(i, 1);
                    obj(i).VELOCITIES(2) = velocs(i, 2);
                    obj(i).VELOCITIES(3) = velocs(i, 3);
                    
                    % RD-ABC/ZZC Variables:
                    obj(i).STAY_COUNTER = 0;
                    obj(i).THRESHOLD_FOUND = false;

                    % RD-ABC Variables:
                    obj(i).LAUNCH = false;                  
                    obj(i).BEE_ROLE = "N/A";
                    obj(i).PERFORMED = false;
                    obj(i).DEST_SEARCH = true;
                    
                    obj(i).CURRENT_SOURCE = 0;                    

                    obj(i).SOURCES_SEARCHED = [];                   
                    
                    obj(i).DISABLED_STEPS = 0;

                    obj(i).ONLOOKER_STANDBY = true;

                    % ZZC Variables:
                    obj(i).ZZC_ROLE = "SELECT";
                    obj(i).ZZC_USE = false;
                    
                    obj(i).PATH_POINTS = [];                   
                    obj(i).CURRENT_POINT = 1;
                    obj(i).SCAN_START = zeros(1, 3);
                    obj(i).DISABLE_STATE = 0;
                    obj(i).START_FOUND = false;
                    obj(i).NEW_SEARCH = true;

                end
            end

        end
    end

end
