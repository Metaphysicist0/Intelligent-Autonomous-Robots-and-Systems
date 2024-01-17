function n_index = node_index(OPEN, xval, yval)
    % This function returns the index of the location of a node in the OPEN list
    %
    % Copyright 2009-2010 The MathWorks, Inc.
    i = 1;
    OPEN_COUNT = size(OPEN, 1);
    while(i <= OPEN_COUNT && (OPEN(i, 2) ~= xval || OPEN(i,3) ~= yval))
        i = i + 1;
    end
    n_index = i;
end