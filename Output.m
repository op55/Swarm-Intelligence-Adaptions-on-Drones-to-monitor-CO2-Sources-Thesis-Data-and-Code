function[] = Output(algorithm, run, bestVal, bestPos, TTFu, TTFs, ORTu, ORTs)
% Outputting values from basic Swarm Intelligence algorithms:

    %% Displaying the results:

    bestPos = bestPos(1) + ", " + bestPos(2) + ", " + bestPos(3);
    
    disp("------------------------------------------------");
    disp(algorithm + " - Run " + run + ":");
    disp("Best Value: " + bestVal);
    disp("Best Position: " + bestPos);
    disp("Time to find Best (Iterations): " + TTFu);
    disp("Time to find Best (Seconds): " + TTFs);
    disp("Overall Run time: " + ORTu + "(" + ORTs + ")");
    
    %% Outputting the results on to a text file:
    
    fileName = algorithm + ".txt";

    % Checking if the given file name already exists: 
    if (exist(fileName, 'file') == 2)
        % Opening file to append information on to it:
        fileID = fopen(fileName, 'a');
    else
        % Creating a new file with headers to identify data:
        fileID = fopen(fileName, 'w');
        fprintf(fileID, 'Run\tPosit\tValue\tTTF (U)\tTTF (S)\tORT (U)\tORT (S)\n');
    end
    
    % Simplifying the names of the data to use:
    algRun = sprintf('%0.1f', run);
    bPosit = "[" + bestPos + "]";
    bValue = sprintf('%0.2f', bestVal);
    TTFu    = sprintf('%0.2f', TTFu);
    TTFs    = sprintf('%0.2f', TTFs);
    maxRunI = sprintf('%0.2f', ORTu);
    maxRunS = sprintf('%0.2f', ORTs);
    
    % Storing all the required data into a cell:
    data = {algRun, bPosit, bValue, TTFu, TTFs, maxRunI, maxRunS};
    
    % Placing the data on to a text file:
    fprintf(fileID, '%s\t%s\t%s\t%s\t%s\t%s\t%s\n',... 
        data{1}, data{2}, data{3}, data{4}, data{5}, data{6}, data{7});
    
    % Closing down the file:
    fclose(fileID);

end

