function [empty, isCollision] = kukaCheckCD(robot, robotCollisionModel, worldCollisionModel, x)

isCollision = [];
N = size(x, 2);
empty = zeros(1,N);


if N == 1
    [isSelfCollision, selfCollisionPairIdx] = exampleHelperManipCheckCollisions(robot, robotCollisionModel, {}, x, true);

    [isWorldCollision,robotCollisionPairIdx,worldCollisionPairIdx] = exampleHelperManipCheckCollisions(robot,robotCollisionModel,worldCollisionModel,x,false);

    isCollision = [double(any([isSelfCollision, isWorldCollision]))];

else
    for i=1:N
        % Check Path
        z = x(:,i);
            
%         show(robot, z);

        [isSelfCollision, selfCollisionPairIdx] = exampleHelperManipCheckCollisions(robot, robotCollisionModel, {}, z, true);

        [isWorldCollision,robotCollisionPairIdx,worldCollisionPairIdx] = exampleHelperManipCheckCollisions(robot,robotCollisionModel,worldCollisionModel,z,false);
       
        isCollision = [isCollision, any([double(any([isSelfCollision, isWorldCollision]))])];
    end
end        
end
