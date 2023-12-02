function randomPositions = generateRandomPositions(N, initialPositions, cornerLat, cornerLon)
    % Validate N to be a multiple of 3
    if mod(N, 3) ~= 0
        error('N must be a multiple of 3');
    end

    % Earth’s radius in meters
    R = 6371000;

    % Preallocate the array for random positions
    randomPositions = zeros(N, 2); % Assuming 2D positions (latitude, longitude)

    % Define bounds of the rectangle
    minLat = min(cornerLat);
    maxLat = max(cornerLat);
    minLon = min(cornerLon);
    maxLon = max(cornerLon);

    % Haversine distance function (element-wise for vectors)
    haversine = @(lat1, lon1, lat2, lon2) 2 * R * asin(sqrt(sin((deg2rad(lat2) - deg2rad(lat1)) / 2) .^ 2 + ...
        cos(deg2rad(lat1)) .* cos(deg2rad(lat2)) .* sin((deg2rad(lon2) - deg2rad(lon1)) / 2) .^ 2));

    % Assign random positions
    for i = 1:N
        isValid = false;
        while ~isValid
            % Generate a random point inside the rectangle
            randLat = minLat + (maxLat - minLat) * rand();
            randLon = minLon + (maxLon - minLon) * rand();

            % Check distance from the closest initial position
            distances = haversine(initialPositions(:,1), initialPositions(:,2), randLat, randLon);
            [minDistance, ~] = min(distances);

            % Check if the distance is within the specified range (0.8 km to 1.2 km)
            if minDistance >= 800 && minDistance <= 1200
                isValid = true;
            end
        end
        % Assign the valid position
        randomPositions(i,:) = [randLat, randLon];
    end
end
