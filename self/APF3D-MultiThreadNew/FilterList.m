function [x,y,z] = FilterList(originalList)
    % 使用索引操作创建新列表
    newList = originalList(1:2:end, :);
    x = newList(:,1);
    y = newList(:,2);
    z = newList(:,3);
end