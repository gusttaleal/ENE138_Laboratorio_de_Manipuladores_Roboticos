%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
clear; clc; close all; disp('Program Started');

vrep=remApi('remoteApi');
vrep.simxFinish(-1);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);
    
    % Handles
    [~, RJ_Arm1]=vrep.simxGetObjectHandle...
        (clientID,'RJ_Arm1',vrep.simx_opmode_blocking);
    
    [~, RJ_Arm2]=vrep.simxGetObjectHandle...
        (clientID,'RJ_Arm2',vrep.simx_opmode_blocking);
    
    [~, EndEffector]=vrep.simxGetObjectHandle...
        (clientID,'EndEffector',vrep.simx_opmode_blocking);
    
    % Ajustado o manipulador da posição inicial:
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm1, 0, vrep.simx_opmode_blocking);
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm2, 0, vrep.simx_opmode_blocking);
    
    % Points
    X = 0.1;
    Y = 0.0;
    
    % 'u' de Theta Up e 'd' de Theta Down
    [TH1u , TH2u , TH1d , TH2d] = InvKinematic(X , Y , 0.1 , 0.1, length(X));
    disp(rad2deg([TH1u ; TH2u]));
    disp(rad2deg([TH1d ; TH2d]));
    
    P = Pos([TH1u ; TH2u] , 0.1 , 0.1)
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm1, TH1u, vrep.simx_opmode_blocking);
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm2, TH2u, vrep.simx_opmode_blocking);
    
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm1, TH1d, vrep.simx_opmode_blocking);
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm2, TH2d, vrep.simx_opmode_blocking);
    
    
    % Angulos de controle
    th1 = 0; th2 = 0;
    for i = 1:1:length(X)
        fprintf('Ponto %d, precione ENTER para continuar.\n', i);
        pause();
        
        % Tempo máximo de iteração
        if( abs(TH1u(i)) > (TH2u(i)) )
            control = TH1u(i);
        else
            control = TH2u(i);
        end
        
        for j = 0 : deg2rad(1) : control
            
            % Ajustado os angulos de controle, faça:
            [~]=vrep.simxSetJointPosition...
                (clientID,RJ_Arm1, TH2u(i), vrep.simx_opmode_blocking);
            
            [~]=vrep.simxSetJointPosition...
                (clientID,RJ_Arm2, TH1u(i), vrep.simx_opmode_blocking);
            
        end
        % Apos objetivo concluido mostre a posição do end effector:
        [~, Pos_measure]=vrep.simxGetObjectPosition...
            (clientID,EndEffector, RJ_Arm1, vrep.simx_opmode_blocking);
        fprintf('Xr: %.4f\tYr: %.4f\tZr: %.4f\n', Pos_measure(1), Pos_measure(2), Pos_measure(3));
    end
    
    % Ajustado o manipulador da posição inicial:
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm1, 0, vrep.simx_opmode_blocking);
    
    [~]=vrep.simxSetJointPosition...
        (clientID,RJ_Arm2, 0, vrep.simx_opmode_blocking);
else
    disp('Failed connecting to remote API server');
end

vrep.simxAddStatusbarMessage(clientID,'Goodbye V-REP!',vrep.simx_opmode_oneshot);
vrep.simxFinish(clientID);
vrep.delete();
disp('Program ended');