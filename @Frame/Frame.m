%close all; f=Frame(2,'V',axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]));
classdef Frame < Geometry
    
    properties (Access = public)
        name_
        scale_
        g_obj_
        ax_
    end   
    
    methods
        function frame = Frame(scale, name, ax, border_color, border_width, fill_color, fill_alpha)
            switch nargin
              case 0
                args{1}=1; 
                args{2}=' ';
                args{3}=axes('XLim',[-5 5],'YLim',[-5 5],'ZLim',[0 1]);
                super_args{1}='r'; super_args{2}=2; super_args{3}='b'; super_args{4}=1;
              case 3
                args{1}=scale; args{2}=name; args{3}=ax;
                super_args{1}='r'; super_args{2}=2; super_args{3}='b'; super_args{4}=1;
              case 7
                args{1}=scale; args{2}=name; args{3}=ax;
                super_args{1}=border_color;
                super_args{2}=border_width;
                super_args{3}=fill_color;
                super_args{4}=fill_alpha;
              otherwise
                error('Invalid number of constructor arguments for Polygon(...)');
            end
            frame = frame@Geometry(super_args{:});
            frame.scale_=args{1};
            frame.name_ = args{2};
            frame.ax_ = args{3};

            frame.g_obj_=zeros(4,1);

            frame.g_obj_(1) = mArrow3(zeros(3,1), [frame.scale_; 0; 0],'color','r','Parent',frame.ax_);%,'stemWidth', frame.scale_,'tipWidth',frame.scale_,'Parent',frame.ax_);
            frame.g_obj_(2) = mArrow3(zeros(3,1), [0; frame.scale_; 0],'color','g','Parent',frame.ax_);%,'stemWidth', frame.scale_,'tipWidth',frame.scale_,'Parent',frame.ax_);
            frame.g_obj_(3) = mArrow3(zeros(3,1), [0; 0; frame.scale_],'color','b','Parent',frame.ax_);%,'stemWidth', frame.scale_,'tipWidth',frame.scale_,'Parent',frame.ax_);
            frame.g_obj_(4) = text(0,0,0,frame.name_);
            
            frame.trans_ = hgtransform('Parent', frame.ax_);
            set(frame.g_obj_,'Parent',frame.trans_);

        end    

        function frame = set.ax_(frame, ax)
            frame.ax_=ax;
        end    
        
        function frame = set.scale_(frame, scale)
            assert(scale > 0);
            frame.scale_=scale;
        end
        
        function frame = set.name_(frame, name)
            assert(ischar(name)); 
            frame.name_=name;
        end 

    end
    
end    