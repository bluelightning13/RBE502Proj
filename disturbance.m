% wind / disturbance function
% same F vector with varyine magnitude
% NOT MOMENT INDUCING
function [X Y Z] = disturbance() %Don
    %F = ma;
    X = rand([3,8]);
    Y = rand([3,8]);
    Z = 0; %assuming no Z forces for initial integration of functions
end
