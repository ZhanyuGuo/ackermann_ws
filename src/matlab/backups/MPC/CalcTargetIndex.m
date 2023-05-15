function idx = CalcTargetIndex(x, y, xRef, yRef)
    %***************计算当前点与目标轨迹最短距离******************%
    %%输入参数：
    %x：当前点x坐标
    %y：当前点y坐标
    %xRef：参考轨迹x坐标
    %yRef：参考轨迹y坐标
    %%输出参数：
    %idx：最近轨迹点索引
    i = 1:length(xRef);
    dist = sqrt((xRef(i) - x) .^ 2 + (yRef(i) - y) .^ 2);
    [~, idx] = min(dist);
end
