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
fprintf('Simulation ID: %d\nMessage: %s\n',outParams.simID, outParams.message);
fprintf('Simulation Time: ');
reverseStr = '';
fileID = fopen('NR60-ran-OMA.txt', 'w+');
powdb=23; R=1;
figure
xdata = [];  % 记录历史x数据的数组
ydata = [];  % 记录历史y数据的数组
% fs=1;                                                       %采样频率
% plottime=simParams.simulationTime*1000/fs;

L=10;
% PHIt = stationManagement.PHIt;
% t=0;
N = simValues.maxID; 
activeIDs = stationManagement.activeIDs;

H=1;
D=2;
C=3;
M=4;
TH=100;
KH=8;
TD=500;
KD=5;
TC =100;
% customer=zeros(N,100000,2);                              %activeID代表车辆数，1000代表是1000个包，2代表如下2个信息
NO=1;                                                           %/*包编号*/  也是产包的总数
gtime=2;                                                        %/*包的产生时刻*/
repeat=3;                                                       %重复总次数
rept=4;                                                         %重复进队的时刻
lambda = 0.0001;          %0.0001=随机消息在1ms内产生一个包的可能性,平均0.1包/秒
k = 1;
pg1 = exp(-lambda) * lambda^k / factorial(k);
wa=0.5;wb=1-wa;

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
Hcustomer=zeros(N,1000,4);
Dcustomer=zeros(N,1000,4);
atnum=zeros(1,N);
st=zeros(N,4);

aoi_consum = zeros(N,1);
ploc = zeros(1,N);
% acon = zeros(1,N);
% Numjsum = zeros(N,1);

stept = zeros(N,1);
steptx = zeros(N,1);
ifreselect = zeros(N,1);
reward = -1*ones(1,N);
episode_reward=0;
episode = 0;
step = 0;

actiondis = randi([1, 3], 1, N); %R*ones(1,N)
actioncon = rand(1,N); %ones(1,N) ;
stationManagement.B = stationManagement.A(actiondis);
stationManagement.RRI= stationManagement.B';
stationManagement.T2autonomousModeTTIs = stationManagement.RRI*1000;
stationManagement.averageTbeacon = stationManagement.RRI;
stationManagement.NbeaconsT = floor(stationManagement.averageTbeacon./phyParams.Tsf);
stationManagement.Nbeacons = appParams.NbeaconsF.*stationManagement.NbeaconsT;
[stationManagement.BRid,~] = BRreassignmentRandom(activeIDs,simParams,timeManagement,sinrManagement,stationManagement,phyParams,appParams);
stationManagement.BRid =stationManagement.BRid(:);

powermax = 10.^((23-30)/10);    %最大功率
phyParams.Ptx_dBm = (powdb+10*log10(actioncon))';      %计算LTE 每MHz的等效辐射功率
phyParams.P_ERP_MHz_CV2X_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_cv2xBR);
phyParams.P_ERP_MHz_CV2X = 10.^((phyParams.Ptx_dBm-30)/10);  % 转换为线性单位
power = (10.^((phyParams.Ptx_dBm - 30)/10))/powermax; %*1000就转换为mW,*5就归一化
energy = zeros(1,N);
while timeManagement.timeNow < simParams.simulationTime*1000+1
    % 接收数据 
    if any(ifreselect)  %做出动作
        idx = find(ifreselect==1);
        actiondis = randi([1, 3], 1, N); %R*ones(1,N);
        actioncon = rand(1,N);    %ones(1,N);
        stationManagement.B(idx) = stationManagement.A(actiondis(idx));
        stationManagement.RRI = stationManagement.B';
        phyParams.Ptx_dBm(idx) = (powdb+10*log10(actioncon(idx)))';      
        phyParams.P_ERP_MHz_CV2X_dBm = (phyParams.Ptx_dBm + phyParams.Gt_dB) - 10*log10(phyParams.BwMHz_cv2xBR);
        phyParams.P_ERP_MHz_CV2X = 10.^((phyParams.P_ERP_MHz_CV2X_dBm-30)/10);  % 转换为线性单位
        power = 10.^((phyParams.Ptx_dBm - 30)/10)/powermax;
        ifreselect = zeros(N,1);
    end
    % The instant and node of the next event is obtained
    % indexEvent is the index of the vector IDvehicle
    % idEvent is the ID of the vehicle of the current event
    % [timeEvent, indexEvent] = min(timeManagement.timeNextEvent(stationManagement.activeIDs));
    % idEvent = stationManagement.activeIDs(indexEvent);

    [minTimeValue,~] = min(timeManagement.timeNextEvent);
    [timeEvent,~] = min(minTimeValue,[],2);                    %时间换算，除以一千
    % indexEvent = indexRow(indexCol);
    % idEvent = activeIDs(indexEvent);
    timeEvent = timeEvent/1000;

    % If the next C-V2X event is earlier than timeEvent, set the time to the
    % C-V2X event
    if timeEvent >= timeManagement.timeNextCV2X - 1e-9
        timeEvent = timeManagement.timeNextCV2X;
        %fprintf('LTE subframe %.6f\n',timeEvent);
    end

    % If the next superframe event (coexistence, method A) is earlier than timeEvent, set the time to the
    % this event
    if timeEvent >= minNextSuperframe  - 1e-9
        timeEvent = minNextSuperframe;
    end
        
    % If timeEvent is later than the next CBR update, set the time
    % to the CBR update
    % if timeEvent >= (timeManagement.timeNextCBRupdate - 1e-9) 
    %     timeEvent = timeManagement.timeNextCBRupdate;
    %     %fprintf('CBR update%.6f\n',timeEvent);
    % end
        
    % If timeEvent is later than the next position update, set the time
    % to the position update
    % With LTE, it must necessarily be done after the end of a subframe and
    % before the next one
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

%     elseif timeEvent == timeManagement.timeNextCBRupdate && ~delayCBRupdate
%         % Part dealing with the channel busy ratio calculation
%         % Done for every station in the system, if the option is active
%         %
%         thisSubInterval = mod(ceil((timeEvent-1e-9)/(simParams.cbrSensingInterval/simParams.cbrSensingIntervalDesynchN))-1,simParams.cbrSensingIntervalDesynchN)+1;
%         %
%         % ITS-G5
%         % CBR and DCC (if active)
%         if ~isempty(stationManagement.activeIDs11p)
%             vehiclesToConsider = stationManagement.activeIDs11p(stationManagement.cbr_subinterval(stationManagement.activeIDs11p)==thisSubInterval);        
%             [timeManagement,stationManagement,stationManagement.cbr11pValues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval-1e-9))] = ...
%                 cbrUpdate11p(timeManagement,vehiclesToConsider,stationManagement,simParams,phyParams,outParams);
% %             %% =========
% %             % Plot figs of related paper, could be commented in other case.
% %             % Please check .../codeForPaper/Zhuofei2023Repetition/fig6
% %             % Only for IEEE 802.11p, highway scenario. 
% %             % log number of replicas
% %             stationManagement.ITSReplicasLog(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval-1e-9)) = stationManagement.ITSNumberOfReplicas(vehiclesToConsider);
% %             stationManagement.positionLog(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval-1e-9)) = positionManagement.XvehicleReal(vehiclesToConsider);
% %             %% =========
%         end
%         % In case of Mitigation method with dynamic slots, also in LTE nodes
%         if simParams.technology==constants.TECH_COEX_STD_INTERF && simParams.coexMethod~=constants.COEX_METHOD_NON && simParams.coex_slotManagement==constants.COEX_SLOT_DYNAMIC && simParams.coex_cbrTotVariant==2
%             vehiclesToConsider = stationManagement.activeIDsCV2X(stationManagement.cbr_subinterval(stationManagement.activeIDsCV2X)==thisSubInterval);
%             [timeManagement,stationManagement,sinrManagement.cbrLTE_coex11ponly(vehiclesToConsider)] = ...
%                 cbrUpdate11p(timeManagement,vehiclesToConsider,stationManagement,simParams,phyParams,outParams);
%         end
% 
%         % LTE-V2X
%         % CBR and DCC (if active)
%         if ~isempty(stationManagement.activeIDsCV2X)
%             vehiclesToConsider = stationManagement.activeIDsCV2X(stationManagement.cbr_subinterval(stationManagement.activeIDsCV2X)==thisSubInterval);
%             [timeManagement,stationManagement,sinrManagement,stationManagement.cbrCV2Xvalues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval)),stationManagement.coex_cbrLteOnlyValues(vehiclesToConsider,ceil(timeEvent/simParams.cbrSensingInterval))] = ...
%                 cbrUpdateCV2X(timeManagement,vehiclesToConsider,stationManagement,positionManagement,sinrManagement,appParams,simParams,phyParams,outParams,outputValues);
%         end
% 
%         timeManagement.timeNextCBRupdate = round(timeManagement.timeNextCBRupdate + (simParams.cbrSensingInterval/simParams.cbrSensingIntervalDesynchN), 10);

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
                    % else
                    %     [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
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
                    % bufferOverflowIndex = find(stationManagement.lt(j,H) >= L);
                    % if ~isempty(bufferOverflowIndex)
                    %     [stationManagement, outputValues] = bufferOverflowLTE(idEvent(bufferOverflowIndex), positionManagement(bufferOverflowIndex), stationManagement(bufferOverflowIndex), phyParams(bufferOverflowIndex), outputValues(bufferOverflowIndex), outParams(bufferOverflowIndex));
                    % end
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
                    % else
                    %     [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
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
                    % bufferOverflowIndex = find(stationManagement.lt(j,D) >= L);
                    % if ~isempty(bufferOverflowIndex)
                    %     [stationManagement, outputValues] = bufferOverflowLTE(idEvent(bufferOverflowIndex), positionManagement(bufferOverflowIndex), stationManagement(bufferOverflowIndex), phyParams(bufferOverflowIndex), outputValues(bufferOverflowIndex), outParams(bufferOverflowIndex));
                    % end
                
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
                    % else
                    %     [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
                    end
                end
            
                pM=rand(1);
                if pM<pg1
                    atnum(j) = atnum(j) +1;
                    timeManagement.timeNextPacket(j,M) =  inf;
                    timeManagement.timeLastPacket(j,M) = t;
                    if stationManagement.lt(j,M)<L                             
                        stationManagement.lt(j,M)=stationManagement.lt(j,M)+1;
                    % else
                    %     [stationManagement,outputValues] = bufferOverflowLTE(idEvent,positionManagement,stationManagement,phyParams,outputValues,outParams);
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

            %acon = acon + nutrate';
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
                ploc(idx) = 0; 
                aoi_consum(idx) = 0;
                atnum(idx) = 0;
            end
            
            % timeManagement.timeNow;
            lt = stationManagement.lt;
            for j = 1:N    %将队首传输以后更新包的位置和队列长度 
                for k = 1:4
                    stjk = st(j,k);
                    if stjk == 0
                        ltvalue = lt(j,k);
                        stationManagement.pckBuffer(j,1:ltvalue,k) = stationManagement.pckBuffer(j,1:ltvalue,k) + 1;  
%                     stationManagement.pckBuffer(:, 1:ltValues(k), k) = stationManagement.pckBuffer(:, 1:ltValues(k), k) + 1;
                    elseif stjk == 1
                        ltvalue = lt(j,k);
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

% Print end of simulation
msg = sprintf('%.1f / %.1fs',simParams.simulationTime,simParams.simulationTime);
fprintf([reverseStr, msg]);

% Number of position updates
simValues.snapshots = positionManagement.NposUpdates;

% Stop stopwatch
outputValues.computationTime = toc;

stationManagement.pckBuffer
end
