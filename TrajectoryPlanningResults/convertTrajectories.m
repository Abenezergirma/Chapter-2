function [latTraj, lonTraj, altTraj] = convertTrajectories(xTraj, yTraj, zTraj, refCoord)
    % Average radius of Earth in meters
    R = 6371000;

    % Define the reference latitude, longitude, and altitude
    refLat = refCoord(1);
    refLon = refCoord(2);
    refAlt = refCoord(3);

    % Conversion factors
    metersPerDegreeLat = 111000;
    metersPerDegreeLon = metersPerDegreeLat * cos(deg2rad(refLat));

    % Preallocate output arrays
    [numUAVs, trajLength] = size(xTraj);
    latTraj = zeros(numUAVs, trajLength);
    lonTraj = zeros(numUAVs, trajLength);
    altTraj = zeros(numUAVs, trajLength);

    % Function to convert x, y back to lat, lon
    xyToLatLon = @(x, y) [y / metersPerDegreeLat + refLat, x / metersPerDegreeLon + refLon];

    % Convert trajectories
    for i = 1:numUAVs
        for j = 1:trajLength
            latLon = xyToLatLon(xTraj(i, j), yTraj(i, j));
            latTraj(i, j) = latLon(1);
            lonTraj(i, j) = latLon(2);
            altTraj(i, j) = refAlt + zTraj(i, j);
        end
    end
end
