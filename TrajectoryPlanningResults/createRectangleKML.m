function [cornersLat, cornersLon] = createRectangleKML(centerCoord)
    % Earth’s radius in meters
    R = 6371000;

    % Convert central latitude and longitude to radians
    centerLatRad = deg2rad(centerCoord(1));
    centerLonRad = deg2rad(centerCoord(2));

    % Assuming a rectangle of 3.5 km² with a 2:1 ratio
    area = 7000000; % 3.5 km² in m²
    width = sqrt(area); % length of the rectangle in meters
    len = width; % width of the rectangle in meters

    % Calculate the latitude and longitude offsets in radians
    latOffset = width / (2 * R);
    lonOffset = len / (2 * R * cos(centerLatRad));

    % Calculate the corner coordinates
    cornersLat = [centerLatRad + latOffset, centerLatRad + latOffset, ...
                  centerLatRad - latOffset, centerLatRad - latOffset, centerLatRad + latOffset];
    cornersLon = [centerLonRad + lonOffset, centerLonRad - lonOffset, ...
                  centerLonRad - lonOffset, centerLonRad + lonOffset, centerLonRad + lonOffset];

    % Convert the coordinates back to degrees
    cornersLat = rad2deg(cornersLat);
    cornersLon = rad2deg(cornersLon);

    % Create KML file
    fileName = 'rectangle_border.kml';
    fileID = fopen(fileName, 'w');

    % Write the KML file header
    fprintf(fileID, '<?xml version="1.0" encoding="UTF-8"?>\n');
    fprintf(fileID, '<kml xmlns="http://www.opengis.net/kml/2.2">\n');
    fprintf(fileID, '<Document>\n');
    fprintf(fileID, '<Style id="blueLine">\n');
    fprintf(fileID, '<LineStyle>\n');
    fprintf(fileID, '<color>FF00BFFF</color>\n'); % Orange color
    fprintf(fileID, '<width>5</width>\n'); % Bold line size
    fprintf(fileID, '</LineStyle>\n');
    fprintf(fileID, '</Style>\n');
    fprintf(fileID, '<Placemark>\n');
    fprintf(fileID, '<styleUrl>#blueLine</styleUrl>\n');
    fprintf(fileID, '<LineString>\n');
    fprintf(fileID, '<altitudeMode>absolute</altitudeMode>\n');
    fprintf(fileID, '<coordinates>\n');

    % Write the corner coordinates
    for i = 1:length(cornersLat)
        fprintf(fileID, '%f,%f,%f\n', cornersLon(i), cornersLat(i), centerCoord(3));
    end

    % Write the KML file footer
    fprintf(fileID, '</coordinates>\n');
    fprintf(fileID, '</LineString>\n');
    fprintf(fileID, '</Placemark>\n');
    fprintf(fileID, '</Document>\n');
    fprintf(fileID, '</kml>');

    % Close the file
    fclose(fileID);
end
