function createKMLFile(latTraj, lonTraj, altTraj, uavNumber)
    % Define a set of colors for the UAVs (in KML color format: aabbggrr)
    colors = {'ff0000ff', 'ff00ff00', 'ffff0000', 'ff00ffff', 'ffff00ff', 'ffffff00', 'ff000000'}; % Red, Green, Blue, Cyan, Magenta, Yellow, Black

    % Open a new file with write permission
    fileName = sprintf('uav_trajectory_%d.kml', uavNumber);
    fileID = fopen(fileName, 'w');

    % Write the header of the KML file
    fprintf(fileID, '<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fileID, '<kml xmlns="http://www.opengis.net/kml/2.2">\n');
    fprintf(fileID, '<Document>\n');
    fprintf(fileID, '<name>UAV %d Trajectory</name>\n', uavNumber);

    % Define style for the line
    fprintf(fileID, '<Style id="uavPathStyle">\n');
    fprintf(fileID, '<LineStyle>\n');
    fprintf(fileID, '<color>%s</color>\n', colors{mod(uavNumber - 1, length(colors)) + 1}); % Cycle through the color array
    fprintf(fileID, '<width>4</width>\n'); % Set the width of the line
    fprintf(fileID, '</LineStyle>\n');
    fprintf(fileID, '</Style>\n');

    % Write the placemark
    fprintf(fileID, '<Placemark>\n');
    fprintf(fileID, '<styleUrl>#uavPathStyle</styleUrl>\n');
    fprintf(fileID, '<LineString>\n');
    fprintf(fileID, '<altitudeMode>absolute</altitudeMode>\n');
    fprintf(fileID, '<coordinates>\n');

    % Combine the trajectory coordinates
    allCoords = [lonTraj(:), latTraj(:), altTraj(:)];

    % Find unique coordinates
    [uniqueCoords, ~, ~] = unique(allCoords, 'rows', 'stable');

    % Write the unique trajectory points to the file
    for i = 1:size(uniqueCoords, 1)
        fprintf(fileID, '%f,%f,%f\n', uniqueCoords(i, 1), uniqueCoords(i, 2), uniqueCoords(i, 3));
    end

    % Write the footer of the KML file
    fprintf(fileID, '</coordinates>\n');
    fprintf(fileID, '</LineString>\n');
    fprintf(fileID, '</Placemark>\n');
    fprintf(fileID, '</Document>\n');
    fprintf(fileID, '</kml>');

    % Close the file
    fclose(fileID);
end
