%close all; p=Polygon([1 2 3 4],rand(4,3),axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]));
classdef Polygon < Geometry
    
    properties (Access = protected)
        faces_
        vertices_
        g_obj_
        ax_
    end   
    
    methods
        function polygon = Polygon(faces, vertices, ax, border_color, border_width, fill_color, fill_alpha)
            switch nargin
              case 0
                args{1}=[1:4]; 
                args{2}=[-1 -1  1 1;
                         -1  1 -1 1;
                         0  0  0 0]'; 
                args{3}=axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
                super_args{1}='r'; super_args{2}=2; super_args{3}='b'; super_args{4}=1;
              case 3
                args{1}=faces; args{2}=vertices; args{3}=ax;
                super_args{1}='r'; super_args{2}=2; super_args{3}='b'; super_args{4}=1;
              case 7
                args{1}=faces; args{2}=vertices; args{3}=ax;
                super_args{1}=border_color;
                super_args{2}=border_width;
                super_args{3}=fill_color;
                super_args{4}=fill_alpha;
              otherwise
                error('Invalid number of constructor arguments for Polygon(...)');
            end
            polygon = polygon@Geometry(super_args{:});
            polygon.faces_ = args{1};
            polygon.vertices_ = args{2};
            polygon.ax_ = args{3};

            polygon.g_obj_=patch('Faces',polygon.faces_,'Vertices',polygon.vertices_,'FaceColor', polygon.fill_color_, 'EdgeColor', polygon.border_color_,'LineWidth', polygon.border_width_, 'FaceAlpha', polygon.fill_alpha_, 'Parent', polygon.ax_);
            polygon.trans_ = hgtransform('Parent', polygon.ax_);
            set(polygon.g_obj_,'Parent',polygon.trans_);
        end    

        function polygon = set.ax_(polygon, ax)
            polygon.ax_=ax;
        end    
        
        function polygon = set.faces_(polygon, faces)
            assert(isvector(faces));
            assert(numel(faces) > 1);
            polygon.faces_=faces;
        end
        
        function polygon = set.vertices_(polygon, vertices)
            assert(size(vertices,1) > 1); 
            assert(size(vertices,2) == 3); 
            polygon.vertices_=vertices;
        end 

    end
    
end    