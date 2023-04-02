function [Obs1,Obs2,Obs3] = PlotObs(n1,n2,n3)
    %% 点障碍物障碍物
    % Obs = rand(n,3)*10;
    % Obs = randi([0,DesX],n,3);               % 随机生成障碍物
    % Obs = [10.5,10.5,10.5];
    % Obs = load('OBS1.mat').Obs;
    % Obs = load('OBS2.mat').Obs;
    % Obs = load('max_step_test_data.mat').Obs;
    %     plot3(Obs(:,1),Obs(:,2),Obs(:,3),'.','MarkerSize',8);
    %% 球体障碍物
    % 定义中心点
    Obs1 = rand(n1,4)*8;
    % 画圆
    for i = 1:n1
        r = rand()*0.5 + 0.5;
        Obs1(i,4) = r;
        PlotSphere(Obs1(i,1),Obs1(i,2),Obs1(i,3),r)
    end

    %% 圆柱体障碍物
    Obs2 = rand(n2,5)*8; % 定义圆柱底部中心位置（平面）
    for i = 1:n2
        r = rand()*0.5 + 1; % 定义半径
        h = rand()*2 + 0.5; % 定义高
%         Obs2(:,3) = 0;
        Obs2(i,4) = r;
        Obs2(i,5) = h;
        PlotCylinder(Obs2(i,1),Obs2(i,2),Obs2(i,3),Obs2(i,4),Obs2(i,5));
    end

    %% 圆锥体障碍物       
    Obs3 = rand(n3,5)*8; % 定义圆柱底部中心位置（平面）
    for i = 1:n3
        r = rand()*0.5 + 1; % 定义半径
        h = rand()*2 + 1; % 定义高
%         Obs3(:,3) = 0;
        Obs3(i,4) = r;
        Obs3(i,5) = h;
        PlotCone(Obs3(i,1),Obs3(i,2),Obs3(i,3),Obs3(i,4),Obs3(i,5));
    end
end
