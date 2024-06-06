% Simplified scenario to use WilabV2Xsim
% Packet size and MCS are set accordingly to utilize the whole channel
% Each transmission uses all the subchannels available.
% NR-V2X is considered for these simulations

% WiLabV2Xsim('help')

close all    % Close all open figures
clear        % Reset variables
clc          % Clear the command window

bit = 4000;
packetSize=bit/8;        % 1000B packet size
nTransm=1;              % Number of transmission for each packet
sizeSubchannel=10;      % Number of Resource Blocks for each subchannel
% Raw = [50, 150, 300];   % Range of Awarness for evaluation of metrics
Raw = 150;
speed=60;               % Average speed
speedStDev=0;           % Standard deviation speed
SCS=15;                 % Subcarrier spacing [kHz]
pKeep=0.8;              % keep probability
periodicity=0.1;        % periodic generation every 100ms
sensingThreshold=-126;  % threshold to detect resources as busy

% Configuration file
configFile = 'Highway3GPP.cfg';


%% NR-V2X PERIODIC GENERATION
for BandMHz=[10]

if BandMHz==10
    MCS=11;
elseif BandMHz==20
    MCS=5;
end    

% for rho=[100 200 300] % number of vehicles/km
for rho=100

simTime=1000;  

outputFolder = sprintf('Output/NRV2X_%dMHz_periodic',BandMHz);

% Launches simulation
WiLabV2Xsim(configFile,'outputFolder',outputFolder,'Technology','5G-V2X','MCS_NR',MCS,'SCS_NR',SCS,'beaconSizeBytes',packetSize,...
    'simulationTime',simTime,'rho',rho,'probResKeep',pKeep,'BwMHz',BandMHz,'vMean',speed,'vStDev',speedStDev,...
    'cv2xNumberOfReplicasMax',nTransm,'allocationPeriod',periodicity,'sizeSubchannel',sizeSubchannel,...
    'powerThresholdAutonomous',sensingThreshold,'Raw',Raw,'FixedPdensity',false,'dcc_active',false,'cbrActive',true)
end
end


%% PLOT of results

figure
hold on
grid on

for iCycle=1:3
    rho=100*iCycle;

    % Loads packet reception ratio output file
    xMode2_periodic=load(outputFolder + "/packet_reception_ratio_"+num2str(iCycle)+"_5G.xls");

    % PRR plot
    % it takes the first column and the last column
    plot(xMode2_periodic(:,1),xMode2_periodic(:,end),'linewidth',2.5,'displayName',"Mode2, periodic generation, vehicles/km=" + num2str(rho))

end
    
    legend()
    title("NR-V2X, " + num2str(BandMHz) + "MHz, MCS=" + num2str(MCS))
    legend('Location','southwest')
    xlabel("Distance [m]")
    ylabel("PRR")
    yline(0.95,'HandleVisibility','off');
