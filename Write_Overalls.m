function[] = Write_Overalls(fileName, runNo, sources, flags, runIter, runTime)
% Write_Overalls - Outputting general algorithm data.
%   Outputting algorithm data, such as overall time and steps taken:
    
    % Checking if the given file name already exists: 
    if (exist(fileName, 'file') == 2)
        % Opening file to append information on to it:
        fileID = fopen(fileName, 'a');
    else
        % Creating a new file with headers to identify data:
        fileID = fopen(fileName, 'w');
        fprintf(fileID, 'Run\t\tSources\t\tFlags\t\tRun time (U)\t\tRun Time (S)\n');
    end
    
    % Data to append:
    data = {runNo, sources, flags, runIter, runTime};
    
    % Appending the data on to the file:
    fprintf(fileID, '%s\t\t\t%s\t\t%s\t\t%s\t\t\t\t%s\n', data{1}, data{2}, data{3}, data{4}, data{5});

    % Closing the file:
    fclose(fileID);

end