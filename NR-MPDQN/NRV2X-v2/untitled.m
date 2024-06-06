% 1. 加载txt文件数据
data = load('NR0.6-40test.txt');
totPHItavg_ind= 1:3:length(data);
totPHItavg = data(totPHItavg_ind);

AoIloc_ind= 2:3:length(data);
AoIloc = data(AoIloc_ind);

Energy_ind= 3:3:length(data);
Energy = data(Energy_ind);

a=mean(totPHItavg)
b=mean(AoIloc)
c=mean(Energy)
% 2. 绘制折线图
figure(1)
plot(totPHItavg);
title('totPHItavg');
xlabel('X');
ylabel('Y');

figure(2)
plot(AoIloc);
title('AoIloc');
xlabel('X');
ylabel('Y');

figure(3)
plot(Energy);
title('Energy');
xlabel('X');
ylabel('Y');

% fileList = dir('*.txt');
% 
% data = struct(); % 创建存储数据的结构体数组
% for i = 1:length(fileList)
%     filename = fileList(i).name;
%     data(i).name = filename; % 设置曲线名称为文件名
%     data(i).values = importdata(filename); % 读取数据，存储在 values 字段中
% end
% 
% figure; % 创建新的图形窗口
% hold on; % 在同一图形窗口中绘制多个曲线
% for i = 1:length(data)
%     plot(data(i).values, 'LineWidth', 1.5); % 绘制曲线并设置线宽
% end
% hold off; % 完成绘制所有曲线
% 
% % 添加图例
% legend({data.name}, 'Location', 'best');
% 
% % 添加坐标轴标签和标题
% xlabel('X轴');
% ylabel('Y轴');
% title('曲线图示例');
