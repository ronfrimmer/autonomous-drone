classdef point
   properties
      x=0;
      y=0;
      d;
   end
   methods
       function d = distance(obj)
         d = sqrt(obj.x^2+obj.y^2);
       end
   end
end