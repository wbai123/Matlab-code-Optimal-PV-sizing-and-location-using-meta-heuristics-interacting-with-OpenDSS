function [S_val, busVoltages] = evaluatePV(FM_pop)
    global DSSObj locationBusMap nDisVar;
    % translate the discrete variables to bus number
    FM_pop(:, 1:nDisVar) = round(FM_pop(:, 1:nDisVar));
    for i = 1 : size(FM_pop, 1)
        FM_pop(i, 1) = locationBusMap(FM_pop(i, 1));
    end

    S_val = nan(size(FM_pop,1),1);
    DSSCircuit = DSSObj.ActiveCircuit; % Access the active circuit
    DSSSolution = DSSCircuit.Solution; % Access the solution object
    busVoltages = nan(size(FM_pop,1), size(DSSCircuit.AllBusVmagPu',1));
    for i = 1 : size(FM_pop,1)
        % DSSObj.Text.Command = 'clear'; % Clear previous circuit
        % DSSObj.Text.Command = 'compile ''G:\My Drive\PES2023\13Bus\IEEE13Nodeckt.dss'''; % Compile the circuit model
        DSSObj.Text.Command = 'Edit PVSystem.PV1 phases=3 bus1=' + string(FM_pop(i,1)) + ' kV=4.8 kVA=' + string(FM_pop(i,2)) + ' irrad=0.983 Pmpp=' + string(FM_pop(2)) + ...
            ' temperature=25 PF=1 effcurve=Myeff P-TCurve=MyPvsT daily=MyIrrad tdaily=MyTemp %cutin=.1 %cutout=.1';


        DSSSolution.Solve; % Solve the power flow
        busVoltages(i, :) = DSSCircuit.AllBusVmagPu; % Get per unit bus voltages
        tmp = busVoltages(i, :);
        % if PF is not solved, then AllBusVmagPu is NaN, so penalize it
        if isnan(tmp(1)) 
            S_val(i) = 1000000;
        else
            upLimitLogic = busVoltages(i, :) - 1.05 > 0;
            lowLimitLogic = busVoltages(i, :) - 0.95 < 0;
            S_val(i) = 0;
            if sum(upLimitLogic)>0
                busVoltagesTmp = busVoltages(i, :);
                S_val(i) = S_val(i) + sum(abs(busVoltagesTmp(upLimitLogic) - 1.05));
            end        
            if sum(lowLimitLogic)>0
                busVoltagesTmp = busVoltages(i, :);
                S_val(i) = S_val(i) + sum(abs(busVoltagesTmp(lowLimitLogic) - 0.95));
            end
            
            S_val(i) = S_val(i) * 1000000 - sum(FM_pop(i,nDisVar+1:end)); % minimize voltage violation while max PV penetration
        end
    end
end