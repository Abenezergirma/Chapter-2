function [initialXY, finalXY] = geoToCartesian(initialPositions, finalPositions, origin)
    % Average radius of Earth in meters
    R = 6371000;
    
    % Conversion factors
    metersPerDegreeLat = 111000;
    metersPerDegreeLon = metersPerDegreeLat * cos(deg2rad(origin(1)));

    % Function to convert lat, lon to x, y
    latLonToXY = @(lat, lon) [(lon - origin(2)) * metersPerDegreeLon, (lat - origin(1)) * metersPerDegreeLat];

    % Convert initial positions
    initialXY = zeros(size(initialPositions, 1), 2);
    for i = 1:size(initialPositions, 1)
        initialXY(i, :) = latLonToXY(initialPositions(i, 1), initialPositions(i, 2));
    end

    % Convert final positions
    finalXY = zeros(size(finalPositions, 1), 2);
    for i = 1:size(finalPositions, 1)
        finalXY(i, :) = latLonToXY(finalPositions(i, 1), finalPositions(i, 2));
    end
end
