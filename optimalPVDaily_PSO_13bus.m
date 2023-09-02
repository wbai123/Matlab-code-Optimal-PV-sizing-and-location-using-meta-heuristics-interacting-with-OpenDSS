clc;
clear;
close all;
format bank;
%% Problem definition
global DSSObj nDisVar nConVar locationBusMap;

% Possible PV bus location: bus 670, 671, 633 - 634 (4.16 - 0.48 kv), 680, 675 692
% Create a new map with int keys and values of int type
locationBusMap = containers.Map('KeyType', 'double', 'ValueType', 'double');
busNumber = [670 671 633 680 675 692];
for i = 1: size(busNumber,2)
    locationBusMap(i) = busNumber(i);
end

DSSObj = actxserver('OpenDSSEngine.DSS'); % Create a new instance of the OpenDSS engine
DSSStart = DSSObj.Start(0); % Start the engine
DSSObj.Text.Command = 'clear'; % Clear previous circuit
DSSObj.Text.Command = 'compile ''G:\My Drive\PES2023\13Bus\IEEE13NodecktDaily.dss'''; % Compile the circuit model
DSSCircuit = DSSObj.ActiveCircuit; % Access the active circuit
DSSSolution = DSSCircuit.Solution; % Access the solution object

nDisVar = 1;
nConVar = 1;
%/* Problem specific variables*/
D = nDisVar + nConVar; %/*The number of parameters of the problem to be optimized*/
varSize=[1,D]; 


%% Parameters of PSO
w=1;
%Inertia coefficient
wDamp=0.80;
%damping factor
c1=0.775;
%personal acceleration coefficient
c2=1.4;
%social acceleration coefficient
%% Initialization
%creating a template for the particles
emptyParticle.Position=[];
emptyParticle.Velocity=[];
emptyParticle.Cost=[];
emptyParticle.Best.Position=[];
emptyParticle.Best.Cost=[];

%creating a global best for refernce
globalBest.Cost=inf;
%initialize population members
runTime = 30;
globalMins=zeros(runTime,1);
ieval = 0;
% Control variables:
lb=[unifrnd(1,1,1,nDisVar), unifrnd(2000,2000,1,nConVar)];
ub=[unifrnd(size(busNumber,2),size(busNumber,2),1,nDisVar), unifrnd(20000,20000,1,nConVar)];
range = ub-lb;
globalFitnessValues = NaN(10,10);
nPops = linspace(10,100,10);
maxCycles = linspace(20,200,10);
for idxPop = 6:6
    for idxCycle = 10:10
        %maximum number of iterations
        maxCycle=maxCycles(idxCycle);  
        %population size (swarm size)
        nPop=nPops(idxPop);
        %creating a population array
        particle=repmat(emptyParticle,nPop,1);
        for r = 1:runTime
        tic
        for i=1:nPop
            %Generate random solution
            particle(i).Position = rand(1,D) .* range + lb;
            %initialize velocity
            particle(i).Velocity=zeros(varSize);
            %Evaluation
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [particle(i).Cost,busVoltagesSol]=evaluatePVDaily(particle(i).Position);
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
            %update the personal cost
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;
            busVoltages(i,:) = busVoltagesSol;
            %update the global best
            if particle(i).Best.Cost<globalBest.Cost
                globalBest=particle(i).Best; %globalBest is the structure that includes Position and Cost
                globalVoltages = busVoltages(i,:);
            end
        end
        
        ieval = ieval + nPop;
        %array to hold best costs at each step
        fitMaxVector=zeros(maxCycle,1);
        %% Main Loop of PSO
        for it=1:maxCycle
            for i=1:nPop
                %update velocity
                particle(i).Velocity=w*particle(i).Velocity+c1*rand(varSize).*(particle(i).Best.Position-particle(i).Position)+c2*rand(varSize).*(globalBest.Position-particle(i).Position);
                %update position
                particle(i).Position=particle(i).Position+particle(i).Velocity;
                
                %  /*if generated parameter value is out of boundaries, it is shifted onto the boundaries*/
                ind=find(particle(i).Position<lb);
                particle(i).Position(ind)=lb(ind);
                ind=find(particle(i).Position>ub);
                particle(i).Position(ind)=ub(ind);
                
                %evaluation
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                [particle(i).Cost,busVoltagesSol]=evaluatePVDaily(particle(i).Position);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
                %update personal best
                if particle(i).Cost<particle(i).Best.Cost
                    particle(i).Best.Position=particle(i).Position;
                    particle(i).Best.Cost=particle(i).Cost;
                    busVoltages(i,:) = busVoltagesSol;
                    %update global best
                    if particle(i).Best.Cost<globalBest.Cost
                        globalBest=particle(i).Best;
                        globalVoltages = busVoltages(i,:);
                    end
                end
            end
            ieval = ieval + nPop;
            %store the best cost value
            fitMaxVector(it)=globalBest.Cost;
            disp(['Iterations' num2str(it) 'BestCost:' num2str(fitMaxVector(it))])
            w=w*wDamp;
        end
        %% Results
        % translate globalParams
        tmpIdx = round(globalBest.Position(1:nDisVar));
        globalParams(1:nDisVar) = locationBusMap(tmpIdx);
        globalMins(r) = globalBest.Cost;
        T(r) = toc;
        end
        globalFitnessValues(idxPop,idxCycle) = globalBest.Cost;
    end
end




% figure;
% plot(fitMaxVector,'LineWidth',2)
% xlabel('Iterations');
% ylabel('Best Cost');
% grid on;