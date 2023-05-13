function n_index = node_index(OPEN,xval,yval)
    %This function returns the index of the location of a node in the list
    %OPEN
    %
    %   Copyright 2009-2010 The MathWorks, Inc.
%     i=1;
%     while(OPEN(i,2) ~= xval || OPEN(i,3) ~= yval )
%         i=i+1;
%     end
%     n_index=i;
    xFind = find((OPEN(:,2)==xval));
    yFind = find((OPEN(:,3)==yval));
    n_index = intersect(xFind,yFind); % 修改了index函数，这个看上去更棒一些
    % if not in OPEN, then reuturn empty
    % if in OPEN, return the inde x of the node in the list OPEN
end