close all; clear; clc;

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

% Handles
[~, RJ_Arm1]=vrep.simxGetObjectHandle(clientID,'RJ_Arm1',vrep.simx_opmode_blocking);
[~, RJ_Arm2]=vrep.simxGetObjectHandle(clientID,'RJ_Arm2',vrep.simx_opmode_blocking);

% Ajustado o manipulador da posição inicial:
vrep.simxSetJointPosition(clientID,RJ_Arm1, deg2rad(0), vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID,RJ_Arm2, deg2rad(0), vrep.simx_opmode_blocking);

% Ângulos Objetivos
TH1 = 0; 
TH2 = 0; 

% Ângulos Iniciais
th1 = 0;   
th2 = 0;

% Ângulo máximo
th = max( abs(TH1) , abs(TH2));

% Passo
step = deg2rad(1);

for i = 0 : step : deg2rad(th)
    % Se TH1 for negativo, faça
    if (TH1 < 0)
        if (th1 > TH1)
            th1 = th1 - step;
        else
            th1 = TH1;
        end
        % Caso contrário:
    else
        if (th1 < TH1)
            th1 = th1 + step;
        else
            th1 = TH1;
        end
    end
    
    % Se TH2 for negativo, faça
    if (TH2 < 0)
        if (th2 > TH2)
            th2 = th2 - step;
        else
            th2 = TH2;
        end
        % Caso contrário:
    else
        if (th2 < TH2)
            th2 = th2 + step;
        else
            th2 = TH2;
        end
    end
    
    % Ajustado o manipulador da posição desejada:
    vrep.simxSetJointPosition(clientID,RJ_Arm1, th1, vrep.simx_opmode_blocking);
    vrep.simxSetJointPosition(clientID,RJ_Arm2, th2, vrep.simx_opmode_blocking);
end
