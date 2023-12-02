function createPackageDeliveryKML(initialPositions, finalPositions, filename)
    % Open a new file with write permission
    fileID = fopen(filename, 'w');

    % Write the header of the KML file
    fprintf(fileID, '<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fileID, '<kml xmlns="http://www.opengis.net/kml/2.2">\n');
    fprintf(fileID, '<Document>\n');
    fprintf(fileID, '<name>Package Delivery Scenario</name>\n');

    % Define styles for initial and final positions
    fprintf(fileID, '<Style id="initialStyle">\n');
    fprintf(fileID, '<IconStyle>\n');
    fprintf(fileID, '<color>FF507FFF</color>\n'); % Red icon for initial positions
    fprintf(fileID, '<Icon><href>http://maps.google.com/mapfiles/kml/shapes/target.png</href></Icon>\n');
    fprintf(fileID, '</IconStyle>\n');
    fprintf(fileID, '</Style>\n');
    fprintf(fileID, '<Style id="finalStyle">\n');
    fprintf(fileID, '<IconStyle>\n');
    fprintf(fileID, '<color>FFED9564</color>\n'); % Green icon for final positions
    fprintf(fileID, '<Icon><href>http://maps.google.com/mapfiles/kml/shapes/ranger_station.png</href></Icon>\n');
    fprintf(fileID, '</IconStyle>\n');
    fprintf(fileID, '</Style>\n');

    % Add placemarks for initial positions
    for i = 1:size(initialPositions, 1)
        fprintf(fileID, '<Placemark>\n');
        fprintf(fileID, '<name>Depot %d</name>\n', i);
        fprintf(fileID, '<styleUrl>#initialStyle</styleUrl>\n');
        fprintf(fileID, '<Point>\n');
        fprintf(fileID, '<coordinates>%f,%f,0</coordinates>\n', initialPositions(i, 2), initialPositions(i, 1));
        fprintf(fileID, '</Point>\n');
        fprintf(fileID, '</Placemark>\n');
    end

    % Add placemarks for final positions
    for i = 1:size(finalPositions, 1)
        fprintf(fileID, '<Placemark>\n');
        fprintf(fileID, '<name>Destination %d</name>\n', i);
        fprintf(fileID, '<styleUrl>#finalStyle</styleUrl>\n');
        fprintf(fileID, '<Point>\n');
        fprintf(fileID, '<coordinates>%f,%f,0</coordinates>\n', finalPositions(i, 2), finalPositions(i, 1));
        fprintf(fileID, '</Point>\n');
        fprintf(fileID, '</Placemark>\n');
    end

    % Write the footer of the KML file
    fprintf(fileID, '</Document>\n');
    fprintf(fileID, '</kml>');

    % Close the file
    fclose(fileID);
end
