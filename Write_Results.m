function[] = Write_Results(fileName, information)
% Write_results - Outputting the results on to a text file.
%   Placing a PP's data into a text file, like position, value, and time:
    
    % Checking if the given file name already exists: 
    if (exist(fileName, 'file') == 2)
        % Opening file to append information on to it:
        fileID = fopen(fileName, 'a');
    else
        % Creating a new file with headers to identify data:
        fileID = fopen(fileName, 'w');
        fprintf(fileID, 'Position\t\tValue\tTime (U)\tTime (S)\tRecharges\t\tArea Time (U)\tArea Time (S)\n');
    end
    
    % Shortening some of the source's data entries:
    position = information(1);
    value    = information(2);
    steps    = information(3);
    time     = information(4);
    charges  = information(5);
    aSteps   = information(6);
    aTime    = information(7);
    
    % Data to append:
    data = {position, value, steps, time, charges, aSteps, aTime};
    
    % Appending the data on to the file:
    fprintf(fileID, '%s\t%s\t\t%s\t%s\t\t%s\t\t\t\t%s\t\t\t%s\n', data{1}, data{2}, data{3}, data{4}, data{5}, data{6}, data{7});

    % Closing the file:
    fclose(fileID);

end