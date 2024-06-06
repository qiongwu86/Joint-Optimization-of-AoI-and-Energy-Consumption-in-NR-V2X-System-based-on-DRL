function [simValues,outputValues,appParams,simParams,phyParams,sinrManagement,outParams,stationManagement] = mainV2X(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement)
% Core function where events are sorted and executed

%% Initialization
[appParams,simParams,phyParams,outParams,simValues,outputValues,...
    sinrManagement,timeManagement,positionManagement,stationManagement] = mainInit(appParams,simParams,phyParams,outParams,simValues,outputValues,positionManagement);


% The variable 'timeNextPrint' is used only for printing purposes
timeNextPrint = 0;

% The variable minNextSuperframe is used in the case of coexistence
minNextSuperframe = min(timeManagement.coex_timeNextSuperframe);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation Cycle
% The simulation ends when the time exceeds the duration of the simulation
% (not really used, since a break inside the cycle will stop the simulation
% earlier)

% Start stopwatch
tic
fprintf('Simulation Time: ');
reverseStr = '';
fileID = fopen('GANR100.txt', 'w+');%20：0.9-0.4 21：1-0.5
L=10;
N = simValues.maxID; 
activeIDs = stationManagement.activeIDs;

% 遗传算法的参数设置
populationSize = N; % 种群大小
chromosomeLength = 10; % 染色体长度
mutationRate = 0.1; % 变异率
crossoverRate = 0.8; % 交叉率
% maxGenerations = 20; % 最大迭代次数
elitismSize = N/10;
tournamentSize = 3;
Generation = 1;
population = randi([0, 1], populationSize, chromosomeLength);%ones(populationSize, chromosomeLength);
for i = 1:populationSize
    if population(i, 1:2) == [1, 1]
        while isequal(population(i, 1:2), [1, 1])  % 确保不是11
            population(i, 1:2) = [0,0];
        end
    end
end
X = decodeChromosome(population);
    
wa=0.6;wb=1-wa;
H=1;
D=2;
C=3;
M=4;
TH=100;
KH=8;
TD=500;
KD=5;
TC =100;
NO=1;                                                           %/*包编号*/  也是产包的总数
gtime=2;                                                        %/*包的产生时刻*/
repeat=3;                                                       %重复总次数
rept=4;                                                         %重复进队的时刻
lambda = 0.0001;          %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
k = 1;
pg1 = exp(-lambda) * lambda^k / factorial(k);
energy = zeros(1,N);
stationManagement.lt=zeros(N,4);                                                  %/*用一个循环函数选择队列的0<b<lt(i,state)
stationManagement.lt(:,3) = round(100*rand(N,1)/10);
a= stationManagement.lt(:,3);
b = round(TC*rand(N,1));
stationManagement.pckBuffer = zeros(N,L,4);
for i = 1:length(a)
    stationManagement.pckBuffer(i,1:a(i),3) = b(i) + flip(TC*((1:a(i))-1));
end
stationManagement.PHIt = repmat(stationManagement.pckBuffer(:, 1, 3), 1, N);%/*接受端信息年龄*/
stationManagement.PHIt = stationManagement.PHIt + round(TC*rand(N,N));
stationManagement.PHIt(1:N+1:end) = 0;
stationManagement.transmittedIDs = [];

Hnum=zeros(N,1);                                         %给H类型计数
Dnum=zeros(N,1);
Hcustomer=zeros(N,100,4);
Dcustomer=zeros(N,100,4);
atnum=zeros(1,N);
st=zeros(N,4);
stept = zeros(N,1);
steptx = zeros(N,1);  
ifreselect = zeros(N,1); 
aoi_consum = zeros(N,1);
reward = -1*ones(1,N);
Fval1 = zeros(1,N);
Fvalnum = zeros(1,N);
% fitness =-1*ones(1,N);
gene=zeros(N,1); 

actiondis = X(:,1)+1;
actioncon = X(:,2)';
stationManagement.B = stationManagement.A(actiondis);
stationManagement.RRI= stationManagement.B';
stationManagement.subframeT2Mode4 = stationManagement.RRI*1000;
stationManagement.averageTbeacon = stationManagement.RRI;
stationManagement.NbeaconsT = floor(stationManagement.averageTbeacon./phyParams.Tsf);
stationManagement.Nbeacons = appParams.NbeaconsF.*stationManagement.NbeaconsT;
[stationManagement.BRid,~] = BRreassignmentRandom(activeIDs,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);
stationManagement.BRid = stationManagement.BRid(:);

powermax = 10.^((23-30)/10);    %最大功率
phyParams.Ptx_dBm = (23+10*log10(actioncon))';      %计算LTE 每MHz的等效辐射功率
phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_cv2xBR);
phyParams.P_ERP_MHz_LTE = 10.^((phyParams.Ptx_dBm-30)/10);  % 转换为线性单位
power = (10.^((phyParams.Ptx_dBm - 30)/10))/powermax; %*1000就转换为mW,*5就归一化

while timeManagement.timeNow < simParams.simulationTime*1000+1
    % 接收数据
    if any(ifreselect)  %做出动作
%         decodedPopulation = decodeChromosome(mutatedPopulation); %bestpopall            
        idx = find(ifreselect==1);
        actiondis = X(:,1)+1;
        actioncon = X(:,2)';        
        stationManagement.B(idx) = stationManagement.A(actiondis(idx));%
        stationManagement.RRI= stationManagement.B';
        phyParams.Ptx_dBm(idx) = (23+10*log10(actioncon(idx)))';      %计算LTE 每MHz的等效辐射功率
        phyParams.P_ERP_MHz_LTE_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_cv2xBR);
        phyParams.P_ERP_MHz_LTE = 10.^((phyParams.P_ERP_MHz_LTE_dBm-30)/10);  % 转换为线性单位
        power = 10.^((phyParams.Ptx_dBm - 30)/10)/powermax;
        ifreselect = zeros(N,1);
        gene(idx) = 1;
    end

    if gene==ones(N,1)
        fitness = Fval1./Fvalnum;
        [bestFitness,~] = max(fitness);
        disp(['Generation: ', num2str(Generation), ', Best Fitness: ', num2str(bestFitness)]);
        [~, idx] = sort(fitness, 'descend');
        tops_idx = idx(1:elitismSize);
        lastpops =  population(tops_idx,:);
        % 选择
        % selectedPopulation = selection(population, fitness);   
        selectedPopulation = tournamentSelection(population, fitness,tournamentSize);
        % 交叉
        offspringPopulation = crossover(selectedPopulation, crossoverRate,elitismSize);    
        % 变异
        mutatedPopulation = mutation(offspringPopulation, mutationRate);
        % 替换当前种群
        population = mutatedPopulation;
        replace_idx = randperm(N, elitismSize);
        population(replace_idx, :) = lastpops(1:elitismSize, :);
        X = decodeChromosome(population);
        Generation = Generation + 1;
        gene=zeros(N,1);
    end

    [minTimeValue,~] = min(timeManagement.timeNextEvent);
    [timeEvent,~] = min(minTimeValue,[],2);                    %时间换算，除以一千
    % indexEvent = indexRow(indexCol);
    % idEvent = activeIDs(indexEvent);
    timeEvent = timeEvent/1000;

    % If the next C-V2X event is earlier than timeEvent, set the time to the
    % C-V2X event
    if timeEvent >= timeManagement.timeNextCV2X - 1e-9
        timeEvent = timeManagement.timeNextCV2X;
        %fprintf('CV2X subframe %.6f\n',timeEvent);
    end

    % If the next superframe event (coexistence, method A) is earlier than timeEvent, set the time to the
    % this event
    if timeEvent >= minNextSuperframe  - 1e-9
        timeEvent = minNextSuperframe;
    end

    if timeEvent >= (timeManagement.timeNextPosUpdate-1e-9) && ...
        (isempty(stationManagement.activeIDsCV2X) || (isfield(timeManagement, "ttiCV2Xstarts") && timeManagement.ttiCV2Xstarts==true))
        timeEvent = timeManagement.timeNextPosUpdate;
    end
    
    % to avoid vechile go out of scenario right before CV2X Tx ending or CBR update
    % special case: timeManagement.timeNextPosUpdate == timeManagement.timeNextCV2X
    if (isfield(timeManagement, "ttiCV2Xstarts") && timeManagement.ttiCV2Xstarts==false) ||...
            timeManagement.timeNextPosUpdate == timeManagement.timeNextCBRupdate
        delayPosUpdate = true;
    else
        delayPosUpdate = false;
    end

    % if the CV2X ending transmission time equals to the timeNextCBRupdate,
    % end the transmission first
    if timeManagement.timeNextCBRupdate == timeManagement.timeNextCV2X &&...
            (isfield(timeManagement, "ttiCV2Xstarts") && timeManagement.ttiCV2Xstarts==false)
        delayCBRupdate = true;
    else
        delayCBRupdate = false;
    end

    % if the CV2X ending transmission time equals to the minNextSuperframe,
    % end the transmission first
    if minNextSuperframe == timeManagement.timeNextCV2X &&...
            (isfield(timeManagement, "ttiCV2Xstarts") && timeManagement.ttiCV2Xstarts==false)
        delay_minNextSuperframe = true;
    else
        delay_minNextSuperframe = false;
    end
    if timeEvent < timeManagement.timeNow/1000
        % error log
        fid_error = fopen(fullfile(outParams.outputFolder,...
            sprintf("error_log_%d.txt",outParams.simID)), "at");
        fprintf(fid_error, sprintf("Time goes back! Stop and check!\nSeed=%d, timeNow=%f, timeEvent=%f\n",...
            simParams.seed, timeManagement.timeNow/1000, timeEvent));
        fclose(fid_error);
    end
    % update timenow, timenow do not go back, deal with float-point-related
    % cases.
    % fixme: need to check
    timeManagement.timeNow = max(timeEvent*1000, timeManagement.timeNow);
    
    % If the time instant exceeds or is equal to the duration of the
    % simulation, the simulation is ended
    if round(timeManagement.timeNow/1000, 10) >= round(simParams.simulationTime, 10)
        break;
    end

    %%
    % Print time to video
    while timeManagement.timeNow/1000 > timeNextPrint  - 1e-9
        reverseStr = printUpdateToVideo(timeManagement.timeNow/1000,simParams.simulationTime,reverseStr);
        timeNextPrint = timeNextPrint + simParams.positionTimeResolution;
    end

    %% Action
    % The action at timeManagement.timeNow depends on the selected event
    % POSITION UPDATE: positions of vehicles are updated
    if timeEvent == timeManagement.timeNextPosUpdate && ~delayPosUpdate
        % DEBUG EVENTS
        % printDebugEvents(timeEvent,'position update',-1);
        
        if isfield(timeManagement,'ttiCV2Xstarts') && timeManagement.ttiCV2Xstarts==false
            % During a position update, some vehicles can enter or exit the
            % scenario; this is not managed if it happens during one
            % subframe
            error('A position update is occurring during the subframe; not allowed by implementation.');
        end
            
        [appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement] = ...
              mainPositionUpdate(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);
        
        % DEBUG IMAGE
        % printDebugImage('position update',timeManagement,stationManagement,positionManagement,simParams,simValues);

        % Set value of next position update
        timeManagement.timeNextPosUpdate = round(timeManagement.timeNextPosUpdate + simParams.positionTimeResolution, 10);
        positionManagement.NposUpdates = positionManagement.NposUpdates+1;

    elseif timeEvent == minNextSuperframe && ~delay_minNextSuperframe
        % only possible in coexistence with mitigation methods
        if simParams.technology~=constants.TECH_COEX_STD_INTERF || simParams.coexMethod==constants.COEX_METHOD_NON
            error('Superframe is only possible with coexistence, Methods A, B, C, F');
        end
        
        % coexistence Methods, superframe boundary
        [timeManagement,stationManagement,sinrManagement,outputValues] = ...
            superframeManagement(timeManagement,stationManagement,simParams,sinrManagement,phyParams,outParams,simValues,outputValues);
                    
        minNextSuperframe=min(timeManagement.coex_timeNextSuperframe(stationManagement.activeIDs));

        % CASE C-V2X
    elseif abs(timeEvent-timeManagement.timeNextCV2X)<1e-8    % timeEvent == timeManagement.timeNextCV2X
        if timeManagement.ttiCV2Xstarts
            % DEBUG EVENTS
            %printDebugEvents(timeEvent,'LTE subframe starts',-1);
            %fprintf('Starts\n');
            if timeManagement.timeNow>0
                [phyParams,simValues,outputValues,sinrManagement,stationManagement,timeManagement] = ...
                    mainCV2XttiEnds(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);  
            end
        % if timeManagement.subframeLTEstarts
            t=timeManagement.timeNow;
            for j=1:N                                         %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
                pH=rand(1);
                if pH<pg1
                    atnum(j) = atnum(j) +1;
                    Hnum(j)=Hnum(j)+1;              %车辆j第Hnum个包产生，共Hnum包
                    Hcustomer(j,Hnum(j),NO)=Hnum(j);
                    Hcustomer(j,Hnum(j),gtime)=t;
                    Hcustomer(j,Hnum(j),repeat)=KH;
                    Hcustomer(j,Hnum(j),rept)=t+TH;
                    %当前新包产生的情况下去计算下一时刻产包时刻，然后将当前时刻的包看作last包，其中addedToGenerationTime在maininit中赋值为0
                    timeManagement.timeNextPacket(j,H) = Hcustomer(j,Hnum(j),rept);
                    timeManagement.timeLastPacket(j,H) = t;
                    if stationManagement.lt(j,H)<L
                        stationManagement.lt(j,H)=stationManagement.lt(j,H)+1;
                    end
                end
                repeatIndex = find(Hcustomer(j,:,repeat) > 0 & abs(t-Hcustomer(j,:,rept))<1e-8);
                if ~isempty(repeatIndex)
                    atnum(j) = atnum(j) +1;
                    timeManagement.timeLastPacket(j,H) = t;
                    Hcustomer(j,repeatIndex,repeat) = Hcustomer(j,repeatIndex,repeat) - 1;
                    repeatMask = Hcustomer(j,repeatIndex,repeat) > 0;
                    Hcustomer(j,repeatIndex(repeatMask),rept) = Hcustomer(j,repeatIndex(repeatMask),rept) + TH;
                    stationManagement.lt(j,H) = stationManagement.lt(j,H) + sum(stationManagement.lt(j,H) < L);
                    nextPacketIndex = find(Hcustomer(j,:,repeat) > 0);
                    if ~isempty(nextPacketIndex)
                        timeManagement.timeNextPacket(j,H) = min(Hcustomer(j,nextPacketIndex,rept));
                    else
                        timeManagement.timeNextPacket(j,H) = inf;
                    end
                end
                                
        
                pD=rand(1);
                if pD<pg1
                    atnum(j) = atnum(j) +1;
                    Dnum(j)=Dnum(j)+1;
                    Dcustomer(j,Dnum(j),NO)=Dnum(j);
                    Dcustomer(j,Dnum(j),gtime)=t;
                    Dcustomer(j,Dnum(j),repeat)=KD;
                    Dcustomer(j,Dnum(j),rept)=t+TD;
                    timeManagement.timeNextPacket(j,D) = Dcustomer(j,Dnum(j),rept);
                    timeManagement.timeLastPacket(j,D) = t;
                    if stationManagement.lt(j,D)<L                           %成功进入队列的包+1
                        stationManagement.lt(j,D)=stationManagement.lt(j,D)+1;
                    end
                end
    
                repeatIndex = find(Dcustomer(j,:,repeat) > 0 & abs(t-Dcustomer(j,:,rept))<1e-8);
                if ~isempty(repeatIndex)
                    atnum(j) = atnum(j) +1;      %   at(j,D) = 1;
                    timeManagement.timeLastPacket(j,D) = t;
                    Dcustomer(j,repeatIndex,repeat) = Dcustomer(j,repeatIndex,repeat) - 1;
                    repeatMask = Dcustomer(j,repeatIndex,repeat) > 0;
                    Dcustomer(j,repeatIndex(repeatMask),rept) = Dcustomer(j,repeatIndex(repeatMask),rept) + TD;
                    stationManagement.lt(j,D) = stationManagement.lt(j,D) + sum(stationManagement.lt(j,D) < L);
            
                    nextPacketIndex = find(Dcustomer(j,:,repeat) > 0);
                    if ~isempty(nextPacketIndex)
                        timeManagement.timeNextPacket(j,D) = min(Dcustomer(j,nextPacketIndex,rept));
                    else
                        timeManagement.timeNextPacket(j,D) = inf;
                    end
                end
    
            
                if  abs(t-timeManagement.timeNextPacket(j,C))<1e-8
                    atnum(j) = atnum(j) +1;
                    timeManagement.timeNextPacket(j,C) = t+TC;
                    timeManagement.timeLastPacket(j,C) = t;
                    if stationManagement.lt(j,C)<L                                        
                        stationManagement.lt(j,C)=stationManagement.lt(j,C)+1;
                    end
                end
            
                pM=rand(1);
                if pM<pg1
                    atnum(j) = atnum(j) +1;
                    timeManagement.timeNextPacket(j,M) =  inf;
                    timeManagement.timeLastPacket(j,M) = t;
                    if stationManagement.lt(j,M)<L                             
                        stationManagement.lt(j,M)=stationManagement.lt(j,M)+1;
                    end
                end      
            end%for j

            qt = stationManagement.lt > 0;

            [sinrManagement,stationManagement,timeManagement,outputValues] = ...
                mainCV2XttiStarts(appParams,phyParams,timeManagement,sinrManagement,stationManagement,simParams,simValues,outParams,outputValues);

            transmittingIDs = stationManagement.transmittingIDsCV2X;
            for j = 1:length(transmittingIDs)
                id = transmittingIDs(j);
                st(id,find(qt(id,:) == 1, 1))=1;          
            end
            timeManagement.ttiCV2Xstarts = false;
            timeManagement.timeNextCV2X = round(timeManagement.timeNextCV2X + (phyParams.TTI - phyParams.TsfGap), 10);

        else

            [phyParams,simValues,outputValues,sinrManagement,stationManagement,timeManagement] = ...
                mainCV2XtransmissionEnds(appParams,simParams,phyParams,outParams,simValues,outputValues,timeManagement,positionManagement,sinrManagement,stationManagement);

            stationManagement.transmittedIDs = stationManagement.transmittingIDsCV2X;
            timeManagement.ttiCV2Xstarts = true;
            timeManagement.timeNextCV2X = round(timeManagement.timeNextCV2X + phyParams.TsfGap, 10);
            timeManagement.timeNextCV2X = round(timeManagement.timeNextCV2X*1e10)/1e10; %去抖动
            timeManagement.timeNow = round(timeManagement.timeNextCV2X*1000);

            transmittingIDs = stationManagement.transmittingIDsCV2X;
            [correctMatrix,stateavg] = findCorrect(transmittingIDs,transmittingIDs,stationManagement.neighborsIDLTE,sinrManagement,stationManagement,positionManagement,phyParams);
            CorrectMatrixRawMax=correctMatrix;
            
            ut = zeros(N,N);    %ut=1代表当前发射且通信成功的情况，st代表当前发射的是哪一个队列
            for j = 1:length(CorrectMatrixRawMax(:,1))   
                ut(CorrectMatrixRawMax(j,1),CorrectMatrixRawMax(j,2))=1;
            end

            distanceReal = positionManagement.distanceReal;
            rawThreshold = phyParams.Raw;%范围内车辆才能算aoi，可注释phyParams.RawMaxLTE
            noAboveThreshold = distanceReal <= rawThreshold;
            Numj = sum(noAboveThreshold, 2)-1;
%             cont = ut.*noAboveThreshold;

            for j = 1:N               %计算接收端aoi
                ut_j = ut(j,:);
                st_j = st(j,:);
                isUt1 = ut_j == 1;
                isSt1 = any(st_j == 1);
                stationManagement.PHIt(j, :) = stationManagement.PHIt(j, :) + 1;
                stationManagement.PHIt(j, isUt1 & isSt1) = stationManagement.pckBuffer(j, 1, find(st_j == 1, 1)) + stateavg(j,3);
                stationManagement.PHIt(j, j) = 0;
            end
            % noAboveThreshold = distanceReal <= phyParams.Raw;
            stationManagement.PHIt = stationManagement.PHIt.*noAboveThreshold;
            ut = ut.*noAboveThreshold;
            nutrate = 1-(sum(ut,2) ./ Numj);   %未通信率   stateavg(j,3)
            nutrate(isnan(nutrate)) = 0;
           %% 时隙变化时向Python发送数据，用于训练
            % resReselectionCounterLTE = stationManagement.resReselectionCounterLTE;    
            %steput < ReCounterLTE0 ,因为队列中没有包的时候是不会发射的，即不会产生功耗
            stept=stept+1;
%             aoi_loct = (sum(sum(stationManagement.pckBuffer(:,:,:), 3), 2) / 40)';
            aoi_cont = sum(stationManagement.PHIt,2)./Numj/1000;
            aoi_cont(isnan(aoi_cont)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
            aoi_consum = aoi_consum + aoi_cont; % /N
            if ~isempty(transmittingIDs)
                ifreselect(transmittingIDs) = (stationManagement.resReselectionCounterCV2X(transmittingIDs) == 1);
                steptx(transmittingIDs) = steptx(transmittingIDs) + 1;
            end

            if any(ifreselect)
                idx = find(ifreselect==1);
                RCrate = (steptx /75)';   %  归一化./stept*1000后1s内最多传输50次： 总功耗为pt*RCt0，但是RCt0经历的时间不同(RRI*λ)，所以需要归一化
                aoi_conavg = aoi_consum(idx)./stept(idx); % /N
                aoi_conavg(isnan(aoi_conavg)) = 0;%aoi_con(Numj(transmittingIDs) ==0 ) = 0
                reward(idx) = - (wa*power(idx)' .* RCrate(idx) + wb*aoi_conavg') ;
                energy(idx) = energy(idx) + power(idx)' .* RCrate(idx);
                reward_nan = any(isnan(reward));
                if sum(reward_nan)>0
                    disp('The reward contains NaN values.');
                end   

                stept(idx) = 0;
                steptx(idx) = 0; 
                aoi_consum(idx) = 0;
                atnum(idx) = 0;
                Fval1(idx) = reward(idx);
                Fvalnum(idx) = 1;
            end 

            lt = stationManagement.lt;
            for j = 1:N    %将队首传输以后更新包的位置和队列长度 
                for k = 1:4
                    stjk = st(j,k);
                    ltvalue = lt(j,k);
                    if stjk == 0
                        stationManagement.pckBuffer(j,1:ltvalue,k) = stationManagement.pckBuffer(j,1:ltvalue,k) + 1;  
                    elseif stjk == 1 
                        stationManagement.pckBuffer(j,1:ltvalue-1,k) = stationManagement.pckBuffer(j,2:ltvalue,k) + 1;
                        stationManagement.pckBuffer(j,ltvalue,k) = 0;
                        stationManagement.lt(j,k) = ltvalue - stjk;
                    end
                end
            end

             %% 测试
            if any(ifreselect)
                totPHItavg = mean(stationManagement.PHIt(:))/1000; % 计算 接收端totPHItavg        
                locsum = nnz(stationManagement.pckBuffer);
                if locsum > 0
                    AoIloc = mean(stationManagement.pckBuffer(:))/1000;        %计算当前时刻系统所有车本地 队列中 数据包的AoI之和
                else
                    AoIloc = 0;
                end
                pow = mean(energy(idx));
                pow(isnan(pow)) = 0;
%                 if ~mod(t,fs) && t>0
                    fprintf(fileID, '%f\n', totPHItavg);
                    fprintf(fileID, '%f\n', AoIloc);
                    fprintf(fileID, '%f\n', pow);
%                 end
                energy(idx) = 0;
            end  
            st=zeros(N,4);
        end
    end
    
    % The next event is selected as the minimum of all values in 'timeNextPacket'
    % and 'timeNextTxRx'
    timeManagement.timeNextEvent = timeManagement.timeNextPacket;
    timeNextEvent = min(timeManagement.timeNextEvent(:));
    if timeNextEvent < timeManagement.timeNow-1e-8 % error check
        format long
        fprintf('next=%f, now=%f\n',min(timeManagement.timeNextEvent(stationManagement.activeIDs)),timeManagement.timeNow);
        error('An event is schedule in the past...');
    end
    
end

fclose(fileID);
% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

stationManagement.pckBuffer
end


% 解码染色体，将二进制编码转换为实际值
function decodedPopulation = decodeChromosome(population)
    decodedPopulation = zeros(size(population, 1), 2);
    decodedPopulation(:, 1) = round(population(:, 1:2) * [2; 1]);
    decodedPopulation(:, 2) = binaryToDecimal(population(:, 3:end)) / (2^(size(population, 2)-2)-1);
end

% 二进制编码转十进制
function decimalValue = binaryToDecimal(binaryValue)
    [~, n] = size(binaryValue);
    binaryValue = fliplr(binaryValue);
    powersOfTwo = 2.^(0:n-1);
    decimalValue = sum(binaryValue .* powersOfTwo, 2);
end

% % 计算适应度的函数
% function fitness = calculateFitness(population)
%     % TODO: 根据问题定义计算适应度
%     % 这里假设适应度是染色体中所有基因值的总和
%     fitness = sum(population, 2);
% end

% 选择操作
function selectedPopulation = selection(population, fitness)
    % 根据适应度进行轮盘赌选择
    fitnessSum = sum(fitness);
    if fitnessSum == 0
        selectionProbabilities = ones(size(fitness)) / numel(fitness);
    else
        selectionProbabilities = fitness / fitnessSum;
    end
%     selectionProbabilities = fitness / fitnessSum;
    selectedIndices = rouletteWheelSelection(selectionProbabilities, numel(fitness));
    selectedPopulation = population(selectedIndices, :);
end

% 轮盘赌选择的一个简化和优化版本
function selectedIndices = rouletteWheelSelection(selectionProbabilities, numSelections)
    cumulativeProbabilities = cumsum(selectionProbabilities);
    randNums = rand(numSelections, 1);
    selectedIndices = zeros(numSelections, 1);
    for i = 1:numSelections
        % 对于每个随机数，找到对应的区间
        index = find(randNums(i) <= cumulativeProbabilities, 1, 'first');
        selectedIndices(i) = index;
    end
end

function selectedPopulation = tournamentSelection(population, fitness, tournamentSize)
    populationSize = size(population, 1);
    selectedPopulation = zeros(populationSize, size(population, 2));
    
    for i = 1:populationSize
        % 随机选择锦标赛参与者
        participants = randperm(populationSize, tournamentSize);
        
        % 从参与者中选择适应度最高的个体进入下一代种群
        [~, winnerIndex] = max(fitness(participants));
        selectedPopulation(i, :) = population(participants(winnerIndex), :);
    end
end



% 交叉操作
function offspringPopulation = crossover(selectedPopulation, crossoverRate, elitismSize)
    offspringPopulation = selectedPopulation;
    numPairs = size(selectedPopulation, 1) / 2;
    
    % 染色体拆分为前两位和后八位
    firstPartLength = 2; % 前两位长度    
    for pair = 1:numPairs  
        % 对前两位染色体确定交叉点并进行交叉操作
        if rand() < crossoverRate
            crossoverPoint = randi([1, firstPartLength]);
            temp = offspringPopulation(pair*2-1, 1:crossoverPoint);
            offspringPopulation(pair*2-1, 1:crossoverPoint) = offspringPopulation(pair*2, 1:crossoverPoint);
            offspringPopulation(pair*2, 1:crossoverPoint) = temp;
        end
        
        % 对后八位染色体确定交叉点并进行交叉操作
        if rand() < crossoverRate
            crossoverPoint = randi([firstPartLength+1, size(selectedPopulation, 2)]);
            temp = offspringPopulation(pair*2-1, crossoverPoint:end);
            offspringPopulation(pair*2-1, crossoverPoint:end) = selectedPopulation(pair*2, crossoverPoint:end);
            offspringPopulation(pair*2, crossoverPoint:end) = temp;
        end
    end
end

% 变异操作
function mutatedPopulation = mutation(offspringPopulation, mutationRate)
    % TODO: 根据变异率进行变异操作，可以使用位翻转、插入删除等
    % 这里使用位翻转
    mutatedPopulation = offspringPopulation;
    mutationMask = rand(size(offspringPopulation)) < mutationRate;
    mutatedPopulation(mutationMask) = ~offspringPopulation(mutationMask);
    %剔除11情况
    for i = 1:size(mutatedPopulation, 1)
        if isequal(mutatedPopulation(i, 1:2), [1, 1])
            % 随机生成00、01或者10
            newPrefix = randi([0, 1], 1, 2);
            while isequal(newPrefix, [1, 1])  % 确保不是11
                newPrefix = [0,1];
            end
            mutatedPopulation(i, 1:2) = newPrefix;
%             mutatedPopulation(i, 1:2) = [0,0];
        end
    end
end
