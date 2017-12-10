% 
% This function takes two joint configurations and the parameters of the
% obstacle as input and calculates whether a collision free path exists
% between them.
% 
% input: q1, q2 -> start and end configuration, respectively. Both are 1x4
%                  vectors.
%        sphereCenter -> 3x1 position of center of sphere
%        r -> radius of sphere
%        rob -> SerialLink class that implements the robot
% output: collision -> binary number that denotes whether this
%                      configuration is in collision or not.
function collision = checkCollision(rob,q1,q2,sphereCenter,r)
    xpoints = linspace(q2{1},q1{1},10);
    ypoints = linspace(q2{2},q1{2},10);
    zpoints = linspace(q2{3},q1{3},10);
    wpoints = linspace(q2{4},q1{4},10);
    apoints = linspace(q2{5},q1{5},10);
    bpoints = linspace(q2{6},q1{6},10);
    c = 0;
    
    for p = 1:10
        inCollision = robotCollision(rob, [xpoints(p),ypoints(p),zpoints(p),wpoints(p),apoints(p),bpoints(p)], sphereCenter, r);
        if inCollision == 1
           c = 1;
    end
    
    collision = c;
end

