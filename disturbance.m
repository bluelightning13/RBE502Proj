%M Hiatt
%This function generates a random small vector to simulate wind force.
%This function will be applied within the controller function
function [X Y Z] = disturbance()
    X = rand([3,8]);
    Y = rand([3,8]);
    Z = 0; %assuming no Z forces for initial integration of functions
end
