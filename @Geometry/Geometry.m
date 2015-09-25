classdef Geometry

    properties
        border_color_
        border_width_
        fill_color_
        fill_alpha_
    end    

    methods
        function geometry = Geometry(border_color, border_width, fill_color, fill_alpha)
            geometry.border_color_='r';
            geometry.border_width_=2;
            geometry.fill_color_='b';
            geometry.fill_alpha_=1;
            
            switch nargin
              case 1
                geometry.border_color_=border_color;
              case 2
                geometry.border_color_=border_color;
                geometry.border_width_=border_width;
              case 3
                geometry.border_color_=border_color;
                geometry.border_width_=border_width;
                geometry.fill_color_=fill_color;
              case 4
                geometry.border_color_=border_color;
                geometry.border_width_=border_width;
                geometry.fill_color_=fill_color;
                geometry.fill_alpha_=fill_alpha;
            end
            
            disp('Constructed Geometry');
        end
    end
end


