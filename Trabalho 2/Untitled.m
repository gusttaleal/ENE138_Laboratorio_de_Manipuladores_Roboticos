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

% Ajustado o manipulador da posição desejada:
vrep.simxSetJointPosition(clientID,RJ_Arm1, deg2rad(0), vrep.simx_opmode_blocking);
vrep.simxSetJointPosition(clientID,RJ_Arm2, deg2rad(90), vrep.simx_opmode_blocking);

L = 0.1;
K = -0.1;
% Points
X = [L 0 0 K K L];
Y = [0 L K 0 K L];

% 'u' de Theta Up e 'd' de Theta Down
[TH1u , ~ , ~ , ~] = InvKinematic(X , Y , L , L, length(X));


%
vrep.simxSetJointPosition(clientID,RJ_Arm1, TH1u(1), vrep.simx_opmode_blocking);
disp( rad2deg( TH1u(1) ) );
pause();

%
vrep.simxSetJointPosition(clientID,RJ_Arm1, TH1u(2), vrep.simx_opmode_blocking);
disp( rad2deg( TH1u(2) ) );
pause();

%
vrep.simxSetJointPosition(clientID,RJ_Arm1, TH1u(3), vrep.simx_opmode_blocking);
disp( rad2deg( TH1u(3) ) );
pause();

%
vrep.simxSetJointPosition(clientID,RJ_Arm1, TH1u(4), vrep.simx_opmode_blocking);
disp( rad2deg( TH1u(4) ) );
pause();

%
vrep.simxSetJointPosition(clientID,RJ_Arm1, TH1u(1), vrep.simx_opmode_blocking);
disp( rad2deg( TH1u(1) ) );
pause();


vrep.simxAddStatusbarMessage(clientID,'Goodbye V-REP!',vrep.simx_opmode_oneshot);
vrep.simxFinish(clientID);
vrep.delete();
disp('Program ended');