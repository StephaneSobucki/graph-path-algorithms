function [noeudsparcourus,chemin] = BreadthFirstSearch(NodeList,RootNode,TargetNode)

noeudsparcourus = 0;

Open = [RootNode];
Closed = [];
found = false;

while true
    if(isempty(Open))
        break
    end
    node = Open(1);
    Closed(end+1) = node;
    Open = Open(2:end);
    noeudsparcourus = noeudsparcourus + 1;
    if(node == TargetNode)
        found = true;
        break
    end
    for x = 1:length(NodeList(node).Neigh)
        if(not(ismember(NodeList(node).Neigh(x),Closed)))
            if(not(ismember(NodeList(node).Neigh(x),Open)))
                Open(end+1) = NodeList(node).Neigh(x);
                NodeList(NodeList(node).Neigh(x)).parent = node;
            end
        end
    end
end
%chemin = Closed;%Pour le parcours de l'arbre (tous les noeuds visites)
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