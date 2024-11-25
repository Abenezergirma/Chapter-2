classdef WindInterpolator
    properties
        wx_interp
        wy_interp
    end
    
    methods
        function obj = WindInterpolator(x, y, wx, wy)
            % Create scattered interpolants for wx and wy
            obj.wx_interp = scatteredInterpolant(x(:), y(:), wx(:), 'linear', 'none');
            obj.wy_interp = scatteredInterpolant(x(:), y(:), wy(:), 'linear', 'none');
        end
        
        function [wx_values, wy_values] = get_wind_at(obj, x_query, y_query)
            % Interpolate wx and wy at the given x_query and y_query coordinates
            wx_values = obj.wx_interp(x_query, y_query);
            wy_values = obj.wy_interp(x_query, y_query);
            
            % Replace NaN values with a constant (e.g., 10)
            wx_values(isnan(wx_values)) = 10;
            wy_values(isnan(wy_values)) = 10;
        end
    end
end
