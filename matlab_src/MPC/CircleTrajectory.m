function [XRef, URef] = CircleTrajectory(xo, yo, r, L)
%***************生成圆形目标轨迹信息******************%
%%输入参数：
    %xo：圆心x轴坐标
    %yo：圆心y轴坐标
    %r：轨迹半径
    %L：小车轴距
%%输出参数：
    %XRef：圆形目标轨迹信息[坐标x（m), 坐标y（m）, 航向（rad）]
    %URef：目标控制量信息[速度（m/s）, 前轮转角（rad）]
    i = 1;

    for theta = - pi + 2 * pi / 200 : 2 * pi / 1000 : pi - 2 * pi / 50
        x(i) = r * cos(theta);
        y(i) = r * sin(theta);
        vRef(i) = 40 /3.6;%速度
        i = i + 1;
    end

    xRef = xo + x;%x坐标
    yRef = yo + y;%y坐标

    dxRef = diff(xRef);
    dxRef(end + 1) = dxRef(end);
    dyRef = diff(yRef);
    dyRef(end + 1) = dyRef(end);
    thetaRef = atan2(dyRef, dxRef);%航向角
    for j = 1 : i - 1
        if (dxRef(j) <= 0) && (dyRef(j) <= 0)
            thetaRef(j) = thetaRef(j) + 2 * pi;
        end
    end

    DTra1 = gradient(yRef) ./ abs(dxRef);%一阶导数
    Dtra2 = del2(yRef) ./ abs(dxRef);%一阶导数
    K = abs(Dtra2) ./ (1 + DTra1 .^ 2) .^ (3 / 2);%曲率
    deltaRef = atan(L * K);%前轮转角

    XRef = [xRef', yRef', thetaRef'];%轨迹信息
    URef =[vRef', deltaRef'];%控制量信息
end