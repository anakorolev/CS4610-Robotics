function qMilestones = prmSearch(graph)
%PRMSEARCH Finds shortest distance to goal
%   Searches for shortest distance to goal positions
%   as long as one exists in this sample

    milestones = py.list({graph{1}{1}.tolist()});
    current = graph{1};
    stop = 0;
    
    while stop < 1
        currP = current{1}.tolist();
        closestP = inf;
        c = inf;
        connectedList = current{2};
        if py.len(connectedList) > 1
            for connectedP = 1:py.len(connectedList)
                exists = 0;
                for m = 1:py.len(milestones)
                    if milestones{m} == connectedList{connectedP}{1}.tolist()
                        exists = 1;
                    end
                end
                if exists ~= 1
                    distance = connectedList{connectedP}{2};
                    if distance < c
                        c = distance;
                        closestP = connectedList{connectedP}{1}.tolist();
                    end
                end
            end
        elseif py.len(connectedList) == 1
            closestP = connectedList{1}{1}.tolist();
        end
        
        milestones.append(closestP);
        
        if currP == graph{py.len(graph)}{1}.tolist()
            stop = 1;
            break
        end
        
        if closestP == inf
            milestones = [];
            break
        end
        
        for p = 1:py.len(graph)
            if graph{p}{1}.tolist() == closestP
                current = graph{p};
                break
            end
        end 
    end
    ms = milestones;
    
    matrix = [];
    
    if milestones ~= []
        for point = 1:py.len(milestones)
            p = milestones{point};
            [rows, cols] = size(matrix);
            shouldAdd = 1;

            if rows >= 1
                for row = 1:rows
                    entireRow = matrix(row,:);
                    pRow = [p{1} p{2} p{3} p{4} p{5} p{6}];
                    if entireRow == pRow
                        shouldAdd = 0;
                        break
                    end
                end
            end

            if shouldAdd == 1
               matrix = [matrix; p{1} p{2} p{3} p{4} p{5} p{6}];
            end

            if p == graph{py.len(graph)}{1}.tolist()
                break
            end

        end
    end
    
    qMilestones = matrix;
end

