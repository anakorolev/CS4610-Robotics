function prmgraph = prmGraph(robot, arrayObjects, qStart, xGoal, sampleNum)
%PRMGRAPH the graph portion of prm algorithm
%   Returns the connected graph of all points
%   as a list of nodes
%   a node is a tuple with:
%       found point
%       list of tuples:
%           connected point
%           connected distance
    
    firstElement = py.tuple({[qStart],py.list()});
    graph = py.list({firstElement});
    
    qDes = robot.ikine(transl(xGoal), 0*ones(1,6), [1 1 1 0 0 0]);
    
    stop = 0;
    
    while stop < sampleNum
        pRand = -pi + (pi - (-pi)).*rand(6,1);
        qRand = [pRand(1), pRand(2), pRand(3), pRand(4), pRand(5), pRand(6)];
        
        % Check if random point is in collision
        for sphere = 1:py.len(arrayObjects)
            a = arrayObjects{sphere}{1}.tolist();
            aT = [a{1};a{2};a{3}];
            pointCollision = robotCollision(robot, qRand, aT, arrayObjects{sphere}{2});
            if pointCollision == 1
                break
            end
        end
        
        % if random point is not in collision add it to graph
        if pointCollision == 0
            graph.append(py.tuple({[qRand], py.list()}));
        end
        
        stop = stop + 1;
    end
    
    graph.append(py.tuple({[qDes], py.list()}));
            
    for point = 1:py.len(graph)
        % finds all points in graph that are within a certain distance
        % of current point
        p = graph{point}{1}.tolist();
        nodes = connectedNodes(graph, p);
        for node = 1:py.len(nodes)
            inCollision = 0;
            for sphere = 1:py.len(arrayObjects)
                a = arrayObjects{sphere}{1}.tolist();
                aT = [a{1};a{2};a{3}];
                p2 = nodes{node}{1}.tolist();
                inCollision = checkCollision(robot, p, p2, aT, arrayObjects{sphere}{2});
                if inCollision == 1
                    break
                end
            end
            if inCollision == 0
                graph{point}{2}.append(nodes{node});
            end
        end
    end
    
    prmgraph = graph;
    
end


function nodes = connectedNodes(g, point)
    list = py.list();
    
    for node = 1:py.len(g)
        array = g{node}{1}.tolist();
        if point ~= array
            distance = sqrt((array{1} - point{1})^2 + (array{2} - point{2})^2 + (array{3} - point{3})^2 + (array{4} - point{4})^2 + (array{5} - point{5})^2 + (array{6} - point{6})^2);
            if distance < 4
                n = [array{1},array{2},array{3},array{4},array{5},array{6}];
                list.append(py.tuple({[n],distance}));
            end
        end
    end
    nodes = list;
end