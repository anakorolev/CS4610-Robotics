% You must run startup_rvc FIRST before running this function.
% DO NOT MODIFY THIS FILE!
% input: questionNum -> Integer between 1 and 3 that denotes question
%                       number to run.
function final_project

    close all;
    
    % set up robot and initial joint configuration
	mdl_puma560
    rob = p560;
	qStart = [0 -0.78 0 -0.78 -0.5 0];
	xGoal = [0.0;0.6;-0.5];
    sampleNum = 20;
    sphereCenter = [.8, 0.4, 0];
    sphereRadius = 0.4;
    sphereCenter2 = [-.5, -.4, -.6];
    sphereRadius2 = 0.25;
    sphereCenter3 = [-.7, 0.5, 0];
    sphereRadius3 = 0.3;
    sphereCenter4 = [0.5, -0.5, 0.4];
    sphereRadius4 = 0.2;
    sphereCenter5 = [-0.4, -0.4, 0.5];
    sphereRadius5 = 0.25;
    
    
    sphere1 = py.tuple({[sphereCenter],[sphereRadius]});
    arrayObjects = py.list({sphere1});
    arrayObjects.append(py.tuple({[sphereCenter2],[sphereRadius2]}));
    arrayObjects.append(py.tuple({[sphereCenter3],[sphereRadius3]}));
    arrayObjects.append(py.tuple({[sphereCenter4],[sphereRadius4]}));
    arrayObjects.append(py.tuple({[sphereCenter5],[sphereRadius5]}));
    
    % plot robot and sphere
    rob.plot(qStart);
    hold on;	
    drawSphere(sphereCenter,sphereRadius);
    drawSphere(sphereCenter2,sphereRadius2);
    drawSphere(sphereCenter3,sphereRadius3);
    drawSphere(sphereCenter4,sphereRadius4);
    drawSphere(sphereCenter5,sphereRadius5);
    
    graph = prmGraph(rob,arrayObjects,qStart,xGoal,sampleNum);
    
    qMilestones = prmSearch(graph)

    % interpolate and plot direct traj from start to goal
    if isempty(qMilestones)
       error('No Path Found: \nNo path found with %d sample points!', sampleNum);
    else
        qTraj = interpMilestones(qMilestones);
        rob.plot(qTraj);
        disp('DONE');
    end
end

function traj = interpMilestones(qMilestones)

    d = 0.05;
    
    traj = [];
    for i=2:size(qMilestones,1)
        
        delta = qMilestones(i,:) - qMilestones(i-1,:);
        m = max(floor(norm(delta) / d),1);
        vec = linspace(0,1,m);
        leg = repmat(delta',1,m) .* repmat(vec,size(delta,2),1) + repmat(qMilestones(i-1,:)',1,m);
        traj = [traj;leg'];
        
    end
end

function qPath = getPath(tree)

    m = 10;
    idx = size(tree,1);
    path = tree(end,1:end-1);
    
    while(idx ~= 1)
        
        curr = tree(idx,1:end-1);
        idx = tree(idx,end);
        next = tree(idx,1:end-1);
        path = [path;[linspace(curr(1),next(1),m)' linspace(curr(2),next(2),m)' linspace(curr(3),next(3),m)' linspace(curr(4),next(4),m)']];
        
    end
    qPath = path(end:-1:1,:);
    
end

function drawSphere(position,diameter)

    [X,Y,Z] = sphere;
    X=X*diameter;
    Y=Y*diameter;
    Z=Z*diameter;
    X=X+position(1);
    Y=Y+position(2);
    Z=Z+position(3);
    surf(X,Y,Z);

end


