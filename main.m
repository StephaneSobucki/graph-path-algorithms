clear all
close all;
addpath("data_structures/","path_finding_algorithm/", "scenario/");
% Initialize the Plan with input values
Obstacle=-1;Target = 0;Robot=1;Space=1;
X_max=10;
Y_max=10;
Plan=CreateScenario(X_max,Y_max);

% Generation of Map_plan2node
 n=0;
 for j=1:Y_max
    for i=1:X_max
        n=n+1;
        Map_plan2node(i,j)=n;
    end
end

% Generation of NodeList
[NodeList] =IncidentList(Map_plan2node,Plan,X_max,Y_max);

% Generation of Nodes
%[Nodes] = IncidentMatrix(Map_plan2node,Plan,X_max,Y_max);%On n'utilise pas
%Nodes


% Identification of TargetNode and RootNode
[target_x,target_y]=find(Plan==-2);
TargetNode=Map_plan2node(target_x,target_y);
[root_x,root_y]=find(Plan==0);
RootNode=Map_plan2node(root_x,root_y);

disp('DFS: ');tic;[DFS_visitedNodes,DFS_Path] = RechercheEnProfondeur(NodeList,RootNode,TargetNode);toc;
disp('BFS: ');tic;[BFS_visitedNodes,BFS_Path] = RechercheEnLargeur(NodeList,RootNode,TargetNode);toc;
disp('A*: ');tic;[Astar_visitedNodes,Astar_Path] = Astar(NodeList,RootNode,TargetNode,Map_plan2node);toc;
disp('A* Manhattan :');tic;[AstarMan_visitedNodes,AstarMan_Path] = AstarMan(NodeList,RootNode,TargetNode,Map_plan2node);toc;

run('CoutVariable')

disp('A* variable cost: ');tic;[Astar_visitedNodes_VariableCost,Astar_Path_VariableCost] = Astar(NodeList,RootNode,TargetNode,Map_plan2node);toc;


found = false;
if(not(isempty(DFS_Path)))
    AnimatePath(DFS_Path,Map_plan2node,'c');
    found = true;
end
if(not(isempty(BFS_Path)))
    AnimatePath(BFS_Path,Map_plan2node,'y');
    found = true;
end
if(not(isempty(Astar_Path)))
    AnimatePath(Astar_Path,Map_plan2node,'r');
    found = true;
end
if(not(isempty(AstarMan_Path)))
    AnimatePath(AstarMan_Path,Map_plan2node,'g--');
    found = true;
end
if(not(isempty(Astar_Path_VariableCost)))
    AnimatePath(Astar_Path_VariableCost,Map_plan2node,'b-.');
    found = true;
end

if(not(found))
    msgbox("Path not found");
else
    legend('DFS','BFS','A*','A* Manhattan','A* variable cost','Location','Best');
end
pause(5);

%Complexité en temps: On crée 4 scénarios différents pour lesquels on va
%stocker le temps d'execution dans une matrice 
temps = zeros(4,5);
figure
T = 0;
for t = 1:T
    clf
    Plan2=CreateScenario(X_max,Y_max);
    % Generation of Map_plan2node
     n=0;
     for j=1:Y_max
        for i=1:X_max
            n=n+1;
            Map_plan2node(i,j)=n;
        end
    end

    % Generation of NodeList
    [NodeList] =IncidentList(Map_plan2node,Plan,X_max,Y_max);

    % Generation of Nodes
    [Nodes] = IncidentMatrix(Map_plan2node,Plan,X_max,Y_max);


    % Identification of TargetNode and RootNode
    [target_x,target_y]=find(Plan==-2);
    TargetNode=Map_plan2node(target_x,target_y);
    [root_x,root_y]=find(Plan==0);
    RootNode=Map_plan2node(root_x,root_y);
    tic;[DFS_visitedNodes,DFS_Path] = RechercheEnProfondeur(NodeList,RootNode,TargetNode); temps(t,1) = toc;
    tic;[BFS_visitedNodes,BFS_Path] = RechercheEnLargeur(NodeList,RootNode,TargetNode);temps(t,2) = toc;
    tic;[Astar_visitedNodes,Astar_Path] = Astar(NodeList,RootNode,TargetNode,Map_plan2node);temps(t,3) = toc;
    tic;[AstarMan_visitedNodes,AstarMan_Path] = AstarMan(NodeList,RootNode,TargetNode,Map_plan2node);temps(t,4) = toc;
    run("CoutVariable");
    tic;[Astar_visitedNodes_VariableCost,Astar_Path_VariableCost] = Astar(NodeList,RootNode,TargetNode,Map_plan2node);temps(t,5) = toc;
end

for j = 1:5
    switch j
        case 1
            fprintf("Temps moyen DFS: %fs\n",mean(temps(:,j)));
        case 2
            fprintf("Temps moyen BFS: %fs\n",mean(temps(:,j)));
        case 3
            fprintf("Temps moyen A*: %fs\n",mean(temps(:,j)));
        case 4
            fprintf("Temps moyen A* Manhattan: %fs\n",mean(temps(:,j)));
        case 5
            fprintf("Temps moyen A* variable cost: %fs\n",mean(temps(:,j)));
    end
end
%Comparaison de la complexité en espace des algorithmes de recherche
%en largeur et en profondeur
load('tree1.mat');
load('tree2.mat');
tree1 = Matrix2List(tree1);
tree2 = Matrix2List(tree2);
array_tree1 = zeros(100,2);
array_tree2 = zeros(100,2);
for i = 1:10000
    start = randi(length(tree1));
    goal = randi(length(tree1));
    if(start > goal)
        [goal,start]=deal(start,goal);
    end
array_tree1(i,1) = RechercheEnLargeur(tree1,start,goal);
array_tree1(i,2) = RechercheEnProfondeur(tree1,start,goal);

array_tree2(i,1) = RechercheEnLargeur(tree2,start,goal);
array_tree2(i,2) = RechercheEnProfondeur(tree2,start,goal);
end
disp('Complexité en espace:');
fprintf("Pour l'arbre 1: \n BFS: %f \n DFS: %f\n",mean(array_tree1(:,1)),mean(array_tree1(:,2)));
fprintf("Pour l'arbre 2: \n BFS: %f \n DFS: %f\n",mean(array_tree2(:,1)),mean(array_tree2(:,2)));
