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
    
    % Points
    X = [-2 5 -7];
    Y = [20 -3 4];
    
    % 'u' de Theta Up e 'd' de Theta Down
    [TH1u , TH2u , TH1d , TH2d] = InvKinematic(X , Y , 10 , 10, length(X));
    
    for i = 1:1:3
        fprintf('Ponto %d, precione ENTER para continuar\n', i);
        pause();
        
        % Tempo máximo de iteração
        if( abs(TH1u) > (TH2u) )
            control = TH1u;
        else
            control = TH2u;
        end
        
        % Angulos de controle
        th1 = 0; th2 = 0;
        
        for j = 0 : deg2rad(1) : control(i)
            
            % Se o angulo objetivo for negativo, faça:
            if( TH1u(i) < 0 )
                % Se o angulo de controle ainda for maior que o de
                % obejetivo, faça:
                if( th1 > TH1u(i) )
                    th1 = -j;
                % Caso contrario:
                else
                    th1 = TH1u(i);
                end
            % Se o angulo objetivo for positivo, faça:
            else
                % Se o angulo de controle ainda for menor que o de
                % obejetivo, faça:
                if( th1 < TH1u(i) )
                    th1 = j;
                % Caso contrario:
                else
                    th1 = TH1u(i);
                end
            end
            
            % O mesmo processo para Th2u
            if( TH2u(i) < 0 )
                if( th2 > TH2u(i) )
                    th2 = -j;
                else
                    th2 = TH2u(i);
                end
            else
                if( th2 < TH2u(i) )
                    th2 = j;
                else
                    th2 = TH2u(i);
                end
            end
            
            % Ajustado os angulos de controle, faça:
            [~]=vrep.simxSetJointPosition...
                (clientID,RJ_Arm1, th1, vrep.simx_opmode_blocking);
            
            [~]=vrep.simxSetJointPosition...
                (clientID,RJ_Arm2, th2, vrep.simx_opmode_blocking);
            
        end
        % Apos objetivo concluido mostre a posição do end effector:
        [~, Pos_measure]=vrep.simxGetObjectPosition...
            (clientID,EndEffector, RJ_Arm1, vrep.simx_opmode_blocking) 
    end
else
    disp('Failed connecting to remote API server');
end

vrep.simxAddStatusbarMessage(clientID,'Goodbye V-REP!',vrep.simx_opmode_oneshot);
vrep.simxFinish(clientID);
vrep.delete();
disp('Program ended');