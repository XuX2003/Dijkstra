function [distance, route] = Dijkstra_by_hand(G, s, t)
    nodes = G.Edges{:, 1};
    weight = G.Edges{:, 2};
    unvistied = zeros(3, 9);
    unvistied(1, :) = 1: 9;
    unvistied(2, :) = inf;
    unvistied(3, :) = -1;
    unvistied(:, s) = [s; 0; s];
    vistied = [];
    while ~isempty(unvistied)
        index = find(unvistied(2, :) == min(unvistied(2, :)), 1);
        goal = unvistied(1, index);
        [Idx, ~] = find(nodes == goal);
        Idx = sort(Idx, 'ascend');
        des = nodes(Idx, :);
        des = reshape(des, 1, []);
        des(des == goal) = [];
        location = [];
        for i = 1: length(des)
            location = [location, find(unvistied(1, :) == des(i))];
        end
        dis = weight(Idx);
    
        for i = 1: length(location)
            if unvistied(2, location(i)) == inf
                unvistied(2, location(i)) = unvistied(2, index) + dis(i);
                unvistied(3, location(i)) = goal;
            else
                if unvistied(2, index) + dis(i) > unvistied(2, location(i))
                    unvistied(2, location(i)) = unvistied(2, location(i));
                    unvistied(3, location(i)) = unvistied(3, location(i));
                else
                    unvistied(2, location(i)) = unvistied(2, index) + dis(i);
                    unvistied(3, location(i)) = goal;
                end
            end
        end
        vistied = [vistied, unvistied(:, index)];
        unvistied(:, index) = [];
        nodes(Idx, :) = [];
        weight(Idx) = [];
        disp(unvistied)
        disp(vistied)
    end
    [~, ind] = sort(vistied(1, :), 'ascend');
    vistied = vistied(:, ind);
    route = [];
    flag = t;
    while flag ~= s
        route = [route, flag];
        INDEX = find(vistied(1, :) == flag);
        flag = vistied(3, INDEX);
    end
    distance = vistied(2, t);
    route = [route, s];
    route = route(end: -1: 1);
end

