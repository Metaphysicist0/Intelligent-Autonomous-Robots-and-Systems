function path = A_star_search(map, MAX_X, MAX_Y)
% This part is about map/obstacle/and other settings
    % pre-process the grid map, add offset
    size_map = size(map, 1);
    Y_offset = 0;
    X_offset = 0;
    
    % Define the 2D grid map array.
    % Obstacle = -1, Target = 0, Start = 1
    MAP = 2 * (ones(MAX_X, MAX_Y));
    
    % Initialize MAP with location of the target
    xval = floor(map(size_map, 1)) + X_offset;
    yval = floor(map(size_map, 2)) + Y_offset;
    xTarget = xval;
    yTarget = yval;
    MAP(xval, yval) = 0;
    
    % Initialize MAP with location of the obstacle
    for i = 2: size_map - 1
        xval = floor(map(i, 1)) + X_offset;
        yval = floor(map(i, 2)) + Y_offset;
        MAP(xval, yval) = -1;
    end 
    
    % Initialize MAP with location of the start point
    xval = floor(map(1, 1)) + X_offset;
    yval = floor(map(1, 2)) + Y_offset;
    xStart = xval;
    yStart = yval;
    MAP(xStart, yStart) = 1;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % LISTS USED FOR ALGORITHM
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % OPEN LIST STRUCTURE
    %--------------------------------------------------------------------------
    % IS ON LIST 1/0 | X val | Y val | Parent X val | Parent Y val | h(n) | g(n) | f(n) |
    %--------------------------------------------------------------------------
    OPEN = [];
    % CLOSED LIST STRUCTURE
    %--------------
    % X val | Y val |
    %--------------
    CLOSED = [];

    % Put all obstacles on the Closed list
    k = 1; % Dummy counter
    for i = 1: MAX_X
        for j = 1: MAX_Y
            if(MAP(i, j) == -1)
                CLOSED(k, 1) = i;
                CLOSED(k, 2) = j;
                k = k + 1;
            end
        end
    end
    CLOSED_COUNT = size(CLOSED, 1);
    % set the starting node as the first node
    xNode = xStart;
    yNode = yStart;
    OPEN_COUNT = 1;
    hn = distance(xNode, yNode, xTarget, yTarget);
    gn = 0;
    OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, hn, gn, hn + gn);
    
    OPEN(OPEN_COUNT, 1) = 0;
    CLOSED_COUNT = CLOSED_COUNT + 1;
    CLOSED(CLOSED_COUNT, 1) = xNode;
    CLOSED(CLOSED_COUNT, 2) = yNode;
    NoPath = 1;

%% This part is your homework
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   while(xNode ~= xTarget || yNode ~= yTarget)
    exp_array = expand_array(xNode, yNode, gn, xTarget, yTarget, CLOSED, MAX_X, MAX_Y);
    EXP_COUNT = size(exp_array,1);
    for i= 1: 1: EXP_COUNT
        n_node = exp_array(i,:);
        n_xval = n_node(1);
        n_yval= n_node(2);
        n_hn = n_node(3);
        n_gn = n_node(4);
        n_fn = n_node(5);
        n_index = node_index(OPEN,n_xval,n_yval);
        if n_index > OPEN_COUNT
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT,:)= insert_open(n_xval, n_yval,xNode, yNode, n_hn, n_gn, n_fn);
        else
            if (OPEN(n_index,1) == 1 && n_fn < OPEN(n_index,7))
                OPEN(n_index,4) = xNode;
                OPEN(n_index,5) = yNode;
                OPEN(n_index,6) = n_hn;
                OPEN(n_index,7) = n_gn;
                OPEN(n_index,8) = n_fn;
            end
        end
    end
    i_min = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
    if i_min == -1
        NoPath=0;
        break;
    end
    OPEN(i_min,1) = 0;
    Node = OPEN(i_min,:);
    xNode = Node(2);
    yNode = Node(3);
    gn = Node(7);
end 

path = [];
if NoPath == 1
    xNode = xTarget;
    yNode = yTarget;
    PATH_COUNT=1;
    path(PATH_COUNT,1)= xNode;
    path(PATH_COUNT,2)= yNode;
    n_index = node_index(OPEN,xNode,yNode);
    xParent = OPEN(n_index,4);
    yParent = OPEN(n_index,5);
    while(xParent ~= xNode || yParent ~= yNode)
        xNode = xParent;
        yNode = yParent;
        PATH_COUNT = PATH_COUNT + 1;
        path(PATH_COUNT,1) = xNode;
        path(PATH_COUNT,2) = yNode;
        n_index = node_index(OPEN,xNode,yNode);
        xParent = OPEN(n_index,4);
        yParent = OPEN(n_index,5);
    end
end
end