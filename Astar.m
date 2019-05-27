function [noeudsparcourus,chemin] = Astar(NodeList,RootNode,TargetNode,Map_plan2node)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Closed = [];
Open = [RootNode];
found = false;

noeudsparcourus = 0;
NodeList(RootNode).g = 0;
[NodeList(TargetNode).x,NodeList(TargetNode).y] = find(Map_plan2node == TargetNode);
[NodeList(RootNode).x,NodeList(RootNode).y] = find(Map_plan2node == RootNode);
NodeList(RootNode).h = sqrt((NodeList(TargetNode).x-NodeList(RootNode).x)^2+(NodeList(TargetNode).y-NodeList(RootNode).y)^2);
while(not(isempty(Open)))
    nbest = Open(1);
    noeudsparcourus = noeudsparcourus + 1;
    Open = Open(2:end);
    Closed(end+1) = nbest;
    if(nbest == TargetNode)
        found = true;
        break;
    end
    for i = 1:length(NodeList(nbest).Neigh)
        if(not(ismember(NodeList(nbest).Neigh(i),Closed)))
            if(not(ismember(NodeList(nbest).Neigh(i),Open)))
                NodeList(NodeList(nbest).Neigh(i)).parent = nbest;
                Open(end+1) = NodeList(nbest).Neigh(i);
                NodeList(NodeList(nbest).Neigh(i)).g = NodeList(nbest).g + NodeList(nbest).K(i);
                [NodeList(NodeList(nbest).Neigh(i)).x,NodeList(NodeList(nbest).Neigh(i)).y]=find(Map_plan2node == NodeList(nbest).Neigh(i));
                NodeList(NodeList(nbest).Neigh(i)).h = sqrt((NodeList(TargetNode).x-NodeList(NodeList(nbest).Neigh(i)).x)^2+(NodeList(TargetNode).y-NodeList(NodeList(nbest).Neigh(i)).y)^2);
            else
                if(NodeList(nbest).g + NodeList(nbest).K(i) < NodeList(NodeList(nbest).Neigh(i)).g)
                    NodeList(NodeList(nbest).Neigh(i)).parent = nbest;
                    NodeList(NodeList(nbest).Neigh(i)).g = NodeList(nbest).g + NodeList(nbest).K(i);
                end
            end
        end
    end
    for i = 1:length(Open)-1
        swapped = false;
        for k = 1:length(Open)-1
            if(NodeList(Open(k)).g + NodeList(Open(k)).h > NodeList(Open(k+1)).g + NodeList(Open(k+1)).h)
                [Open(k+1),Open(k)]=deal(Open(k),Open(k+1));
                swapped = true;
            elseif(NodeList(Open(k)).g + NodeList(Open(k)).h == NodeList(Open(k+1)).g + NodeList(Open(k+1)).h)
                if(NodeList(Open(k)).g < NodeList(Open(k+1)).g)
                    [Open(k+1),Open(k)] = deal(Open(k),Open(k+1));
                    swapped = true;
                end
            end            
        end
        if(not(swapped))
            break
        end
    end
end


if(found)
    chemin = [TargetNode];
    %Parcours inverse de closed pour retrouver le chemin
    %à commenter pour le parcours de l'arbre
    for i = length(Closed):-1:2
        if(NodeList(Closed(i)).parent == Closed(i-1))
            chemin = [Closed(i-1),chemin];
        else
            Closed(i-1) = Closed(i);
        end
    end
else
    chemin = [];
    disp("Path not found");
end


end