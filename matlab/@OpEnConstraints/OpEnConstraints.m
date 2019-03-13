classdef OpEnConstraints
    %OPENCONSTRAINTS fixed (non-parametric) constraints for OpEN
    %   Parametric constraints are not implemented yet
    
    properties (Access = private)
        type;
        params;
    end
    
    methods (Access = public)
        
        function t = get_type(obj)
            t = obj.type;
        end
        
        function p = get_params(obj)
            p = obj.params;
        end
        
    end
    
    methods (Access = private)
        function obj = OpEnConstraints()
        end
    end
    
    methods (Static)
        
        function o = make_ball_at_origin(radius)
            o = OpEnConstraints();
            o.type = 'ball';
            o.params.radius = radius;
        end
        
        function o = make_ball(radius, centre)
            o = OpEnConstraints();
            o.type = 'ball';
            o.params.radius = radius;
            o.params.centre = centre;
        end
        
        function o = make_no_constraints()
            o = OpEnConstraints();
            o.type = 'no_constraints';
            o.params = [];
        end
    end
    
end

