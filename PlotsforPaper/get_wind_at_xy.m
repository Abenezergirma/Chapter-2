function [wx_values, wy_values] = get_wind_at_xy(x_query, y_query)
    % Efficient wind interpolation function using scatteredInterpolant
    persistent wx_interp wy_interp
    
    % Check if interpolants are already created, load data if needed
    if isempty(wx_interp) || isempty(wy_interp)
        % Load the data from workspace or file
        load('filtered_wind_data.mat', 'x', 'y', 'x_velocity', 'y_velocity');
        
        % Create scattered interpolants for wx and wy
        wx_interp = scatteredInterpolant(x(:), y(:), x_velocity(:), 'linear', 'none');
        wy_interp = scatteredInterpolant(x(:), y(:), y_velocity(:), 'linear', 'none');
    end
    
    % Interpolate wind data
    wx_values = wx_interp(x_query, y_query);
    wy_values = wy_interp(x_query, y_query);
end



