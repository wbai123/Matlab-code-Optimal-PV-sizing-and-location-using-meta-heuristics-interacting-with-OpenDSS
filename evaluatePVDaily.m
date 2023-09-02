function [S_val, busVoltages] = evaluatePVDaily(FM_pop)
    global DSSObj locationBusMap nDisVar;
    % translate the discrete variables to bus number
    FM_pop(:, 1:nDisVar) = round(FM_pop(:, 1:nDisVar));
    for i = 1 : size(FM_pop, 1)
        FM_pop(i, 1) = locationBusMap(FM_pop(i, 1));
    end

    S_val = nan(size(FM_pop,1),1);
    DSSCircuit = DSSObj.ActiveCircuit; % Access the active circuit
    DSSSolution = DSSCircuit.Solution; % Access the solution object
    busVoltages = nan(size(FM_pop,1), size(DSSCircuit.AllBusVmagPu',1)*24);
    for i = 1 : size(FM_pop,1)
        DSSObj.Text.Command = 'Edit PVSystem.PV1 phases=3 bus1=' + string(FM_pop(i,1)) + ' kV=4.8 kVA=' + string(FM_pop(i,2)) + ' irrad=0.983 Pmpp=' + string(FM_pop(2)) + ...
            ' temperature=25 PF=1 effcurve=Myeff P-TCurve=MyPvsT daily=MyIrrad tdaily=MyTemp %cutin=.1 %cutout=.1';

        S_val(i) = 0;
        % loop 24 times to solve at each hour and retrieve the bus voltages
        % at that specific hour
        for j = 1 : 24
            DSSSolution.Solve; % Solve the power flow
            busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j) = DSSCircuit.AllBusVmagPu; % Get per unit bus voltages
            tmp = busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j);
            % if PF is not solved, then AllBusVmagPu is NaN, so penalize it
            if isnan(tmp(1)) 
                S_val(i) = 1000000;
            else
                upLimitLogic = busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j) - 1.05 > 0;
                lowLimitLogic = busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j) - 0.95 < 0;
                if sum(upLimitLogic)>0
                    busVoltagesTmp = busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j);
                    S_val_tmp = sum(abs(busVoltagesTmp(upLimitLogic) - 1.05));
                    S_val(i) = S_val(i) + S_val_tmp;
                end        
                if sum(lowLimitLogic)>0
                    busVoltagesTmp = busVoltages(i, (j-1)*size(DSSCircuit.AllBusVmagPu',1)+1 : size(DSSCircuit.AllBusVmagPu',1)*j);
                    S_val_tmp = sum(abs(busVoltagesTmp(lowLimitLogic) - 0.95));
                    S_val(i) = S_val(i) + S_val_tmp;
                end
            end
        end
        S_val(i) = S_val(i)*1000000 - sum(FM_pop(i,nDisVar+1:end)); % minimize voltage violation while max PV penetration
    end
end