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

th1 = -60;
th2 = 120;
th = max( abs(th1) , abs(th2))

if th1 > 0
    for i = 0 : deg2rad(1) : deg2rad(th1)
        % Ajustado o manipulador da posição desejada:
        vrep.simxSetJointPosition(clientID,RJ_Arm1, i, vrep.simx_opmode_blocking);        
    end
else
    for i = 0 : -deg2rad(1) : deg2rad(th1)
        % Ajustado o manipulador da posição desejada:
        vrep.simxSetJointPosition(clientID,RJ_Arm1, i, vrep.simx_opmode_blocking);        
    end
end

if th2 > 0
for i = 0 : deg2rad(1) : deg2rad(th2)
    % Ajustado o manipulador da posição desejada:
    vrep.simxSetJointPosition(clientID,RJ_Arm2, i, vrep.simx_opmode_blocking);    
end
else
    for i = 0 : -deg2rad(1) : deg2rad(th2)
    % Ajustado o manipulador da posição desejada:
    vrep.simxSetJointPosition(clientID,RJ_Arm2, i, vrep.simx_opmode_blocking);    
    end
end