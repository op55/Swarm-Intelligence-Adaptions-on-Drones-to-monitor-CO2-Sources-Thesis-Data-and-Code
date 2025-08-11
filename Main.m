%% Setting up the Simulation:

clc; clear; close all;


%% Constant Variables:

% General Constants:
ITERATIONS = 4000;      % The number of iterations.
NUM_DIMENSIONS = 3;     % The number of dimensions to use.
NUM_DRONES = 4;         % Total number of drones to use.
MIN_ALTITUDE = 13;      % The minimum altitude for drones to search.
STATE_CHECK = 400;      % Iterations to reset drones on RD Algorithms.

% Scenario Constants:
AREA_SIZE = 250;        % The size of the area for its X and Y dimensions.
ALTITUDE = 30;          % The size of the Z-axis for the matrix.
BASE_SIZE = 20;         % The size of the drone base.

% Battery Constants, used in the RD Algorithms:
LOW_BATTERY = 20;       % Life of battery for drones to go back to base.
TOTAL_BATTERIES = 3;    % How many batteries each drone has.
DECREASE_RATIO = 0.023; % The base to decrease a drone's battery.
REPLACE_TIME = 50;      % Iterations to replace a drone battery.

% Plume Constants:
MIN_POLLUTION  = 400.0; % Minimum pollution to be displayed in plots.
DECAY_RATE     = 0.01;  % Controls the decay rate over time.
WIND_INFLUENCE = 2.0;   % Added to amplify wind effect.


%% The General Parameters for use in all Scenarios:

% Drone Parameters:
upMove = [2, 2, 1];     % The maximum number of spaces a drone can move.
loMove = [1, 1, 0];     % The minumum number of spaces a drone can move.

% Plume Parameters:
scalingFactor = 1.5;    % Controls plume spread for the X and Y dimensions.
zScale = 0.4;           % Z Dimension has reduced spread intensities.
seedNum = 3;            % Sets randonmess for scenario generation.

% Smoke Parameters:
density = 20000;         % How many particles are in each plume.
sigma = 10;              % Controls in intensity of the plume spreads.

% The size of the origin areas:
originRadius = [40, 40, 2];    


%% Defining the Pollution Sources for each Scenario:

% One Source:
ORIGIN_POS_1  = [125, 125, 20];  % Placed in the center.
PPM_MAX_1     = 1000;            % The max PPM for one particle.
START_TIMES_1 = 100;             % Iteration when plume appears.
STOP_TIMES_1  = 3800;            % Iteration when plume is turned off.
MIN_MOVE_1    = 4;               % Prevent plumes from going below their origin altitude.

% Controls the size of the individual plume(s):
SPREAD_PARAMS_1 = [0.2, 3.0, 0.1,  0.2, 3.0, 0.1,  0.2, 0.3, 0.01];

% Controls the velocity of the plume(s):
VELOCITIES_1 = [0, 0, 0];

% Two Sources:
ORIGIN_POS_2  = [170, 230, 16; 120, 100, 17];
PPM_MAX_2     = [1000, 900];
START_TIMES_2 = [50, 2000];
STOP_TIMES_2  = [2500, 3800];
MIN_MOVE_2    = [3, 2];
SPREAD_PARAMS_2 = [
    0.2, 1.0, 0.1,  0.2, 1.0, 0.1,  0.2, 0.3, 0.01;...
    0.2, 2.0, 0.2,  0.2, 1.5, 0.1,  0.2, 0.4, 0.01
];
VELOCITIES_2 = [0, 0, 0;  0, 0, 0];

% Three Sources:
ORIGIN_POS_3  = [40, 200, 20; 100, 160, 24; 200, 60, 13];
PPM_MAX_3     = [1000, 900, 800];
START_TIMES_3 = [10, 2200, 1800];
STOP_TIMES_3  = [1900, 3500, 2500];
MIN_MOVE_3    = [4, 6, 2];
SPREAD_PARAMS_3 = [   
    0.2, 1.0, 0.1,  0.2, 2.0, 0.1,  0.2, 0.4, 0.1;...
    0.1, 5.0, 0.1,  0.2, 5.0, 0.1,  0.3, 0.5, 0.1;...
    0.1, 2.0, 0.1,  0.1, 1.5, 0.1,  0.2, 0.2, 0.1
];
VELOCITIES_3 = [0, 0, 0;  0, 0, 0;  -0.2, 0, 0];

% Four Sources:
ORIGIN_POS_4    = [60, 75, 17; 190, 75, 20; 60, 175, 17; 190, 175, 20];
PPM_MAX_4       = [900, 800, 800, 700];
START_TIMES_4   = [20, 1000, 1300, 1700];
STOP_TIMES_4    = [3400, 2800, 3100, 3700];
MIN_MOVE_4      = [2, 3, 2, 3];
SPREAD_PARAMS_4 = [
    0.2, 1.4, 0.1,  0.2, 0.7, 0.1,  0.2, 0.3, 0.1;...
    0.2, 1.3, 0.1,  0.2, 0.9, 0.1,  0.2, 0.4, 0.1;...
    0.2, 1.4, 0.1,  0.2, 0.7, 0.1,  0.2, 0.2, 0.1;...
    0.2, 1.5, 0.1,  0.2, 0.6, 0.1,  0.2, 0.3, 0.1
];
VELOCITIES_4 = [0, 0, 0;  0, 0, 0;  0, 0, 0;  0, 0, 0];


% The choice for the scenario to use (all numbers must match):
originPos    = ORIGIN_POS_1;
ppmMax       = PPM_MAX_1;
startTimes   = START_TIMES_1;
stopTimes    = STOP_TIMES_1;
minMove      = MIN_MOVE_1;
spreadParams = SPREAD_PARAMS_1;
velocities   = VELOCITIES_1;

SEV_POLLUTION = (min(ppmMax) - 15); % Minimum pollution for some algorithms' hotspots.


%% Defining the Wind Conditions for each Scenario:

% Scenario 1 Winds:
INITIAL_WINDS_1 = [2,  2, 0];   % North-East Wind at the start (X, Y, Z).
FINAL_WINDS_1   = [2, -2, 0];   % South-East Wind at the end (X, Y, Z).
TRANS_START_1   = 2100;         % Iteration when the wind starts to change.
TRANS_DURAT_1   = 200;          % Num of iterations for the wind to change.

% Scenario 2 Winds:
INITIAL_WINDS_2 = [-2, 0, 0];
FINAL_WINDS_2   = [0, 2, 0];
TRANS_START_2   = 2500;
TRANS_DURAT_2   = 300;

% Scenario 3 Winds:
INITIAL_WINDS_3 = [0, 1, 0];
FINAL_WINDS_3   = [0, -1, 0];
TRANS_START_3   = 2000;
TRANS_DURAT_3   = 300;

% Scenario 4 Winds:
INITIAL_WINDS_4 = [1, 0, 0];
FINAL_WINDS_4   = [-1, -1, 0];
TRANS_START_4   = 2500;
TRANS_DURAT_4   = 200;

% The choice for wind to use (all numbers must match):
initialWinds       = INITIAL_WINDS_1;
finalWinds         = FINAL_WINDS_1;
transitionStart    = TRANS_START_1;
transitionDuration = TRANS_DURAT_1;


%% Setting up the Arrays for Parameter Usage:

constants   = [ITERATIONS, NUM_DIMENSIONS, NUM_DRONES];
searches    = [MIN_ALTITUDE, STATE_CHECK];
scenarios   = [MIN_POLLUTION, SEV_POLLUTION, DECAY_RATE, WIND_INFLUENCE];
batteries   = [LOW_BATTERY, TOTAL_BATTERIES, DECREASE_RATIO, REPLACE_TIME];
areas       = [AREA_SIZE, AREA_SIZE, ALTITUDE];

smokes      = [density, sigma];
scales      = [scalingFactor, zScale, seedNum]; 
plumes      = [ppmMax; startTimes; stopTimes; minMove];
winds       = [initialWinds; finalWinds];
transitions = [transitionStart; transitionDuration];
movements   = [upMove; loMove];

% Setting up all of the arrays into a cell for viewing simplicity:
PARAMETERS = {constants; searches; scenarios; batteries; areas; smokes; scales; plumes; originPos; spreadParams; velocities; winds; transitions; movements};
ENVIRONMENT = rand(areas) * 50;


%% Setting up the drones for the RD Algorithms:

[drones, locations] = Place_Drones(ITERATIONS, NUM_DRONES, AREA_SIZE, AREA_SIZE, ALTITUDE, BASE_SIZE);


%% Running each Algorithm:

for run = 1:25

    % Original (O) SI Algorithms:
    cd 'O-PSO'\
        O_PSO_Main(PARAMETERS, ENVIRONMENT, run);
    cd ..\

    cd 'O-FA'\
        O_FA_Main(PARAMETERS, ENVIRONMENT, run);
    cd ..\

    cd 'O-ABC'\
        O_ABC_Main(PARAMETERS, ENVIRONMENT, run);
    cd ..\


    % Modified (M) SI Algorithms:
    cd 'M-PSO'\
        M_PSO_Main(PARAMETERS, ENVIRONMENT, drones, locations, run);
    cd ..\

    cd 'M-FA'\
        M_FA_Main(PARAMETERS, ENVIRONMENT, drones, locations, run);
    cd ..\

    cd 'M-ABC'\
        M_ABC_Main(PARAMETERS, ENVIRONMENT, drones, locations, run);
    cd ..\


    % Custom Zig-Zag Climber (ZZC) Algorithm:
    cd 'ZZC'\
        ZZC_Main(PARAMETERS, ENVIRONMENT, drones, locations, run);
    cd ..\

end


% ------------------------------------------------------------------------
% Testing Ground for Algorithm Functions:
% ------------------------------------------------------------------------

saveInterval = 50;
saveIndex = 1;
storedFrames = floor(ITERATIONS / saveInterval);
envStack = zeros(areas(1), areas(2), areas(3), storedFrames);

for i = 1:ITERATIONS

    [ENVIRONMENT, originPos] = Move_Pollution( ...
        i, ENVIRONMENT, areas, smokes, scales, DECAY_RATE, WIND_INFLUENCE,...
        originPos, ppmMax, startTimes, stopTimes, minMove, spreadParams, velocities,...
        initialWinds, finalWinds, transitionStart, transitionDuration...
    );

    if (mod(i, saveInterval) == 0)

        envStack(:, :, :, saveIndex) = ENVIRONMENT;

        % 3D Visualization
        [X, Y, Z] = ind2sub(size(ENVIRONMENT), find(ENVIRONMENT > MIN_POLLUTION));
        C = ENVIRONMENT(sub2ind(size(ENVIRONMENT), X, Y, Z));

        scatter3(X, Y, Z, 20, C, 'filled');
        colorbar;
        clim([400, 1000]);
        title(['3D Pollution Plume - Frame ' num2str(i)]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        axis([0, areas(1), 0, areas(2), 0, areas(3)]);

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

        view(2); grid on;
        drawnow;

        saveIndex = saveIndex + 1;
    end
end
