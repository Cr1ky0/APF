function [Obs] = PlotObs(n)
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
    Obs = rand(n,4)*8 + 1;
    % 画圆
    for i = 1:n
        r = rand()*0.5 + 0.2;
        Obs(i,4) = r;
        PlotSphere(Obs(i,1),Obs(i,2),Obs(i,3),r)
    end
end
