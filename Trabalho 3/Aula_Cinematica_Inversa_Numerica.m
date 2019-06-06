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
    
    % Ângulos iniciais das juntas
    TH = [ 0 ; 0 ];
    t1 = TH(1);
    t2 = TH(2);
    
    % Tamanho dos links
    L1 = 0.1;   L2 = 0.1;
    
    % Conjunto de pontos objetivo
    G = [ 0.1 ; 0.1 ];
    
    % Constantes de treinamento
    Eta = 1e-3;
    Csi = 1e-6;
    
    % Função para aquisição da posição atual do manipulador
    P = Pos(TH, L1, L2);
    
    % Cálculo do erro médio quadratico
    EQM = 0.5 * ( G - P )' * ( G - P );
    
    while( EQM > Csi )
        %
        dEQM = -( G - P );
        
        % Aquisição do Jacobiano da função f(Th1 , Th2)
        J = Jacobinano_planar(TH, L1, L2);
        
        %
        dTH = pinv(J) * dEQM; % Problema aqui
        
        % Atualização dos ângulos thetas
        TH = TH - ( Eta * dTH );
        
        % Função para aquisição da posição atual do manipulador
        P = Pos(TH, L1, L2);
        
        % Cálculo do erro médio quadratico
        EQM = 0.5 * ( G - P )' * ( G - P );
    end
    
    rad2deg(TH)
    
    % Correção do ângulo TH1 para o intervalo [ 0° , 360° ]
    TH(1) = mod( TH(1) , 2 * pi );
    if(TH(1) > pi)
        TH(1) = TH(1) - ( 2 * pi );
    end
    
    % Correção do ângulo TH2 para o intervalo [ 0° , 360° ]
    TH(2) = mod( TH(2) , 2 * pi );
    if(TH(2) > pi)
        TH(2) = TH(2) - ( 2 * pi );
    end
    
    rad2deg(TH)
    
    % Ângulo máximo
    th = max( abs(TH(1)) , abs(TH(2)));
    
    % Passo
    step = deg2rad(1);
    
    % Ângulos Iniciais
    th1 = 0;
    th2 = 0;
    
    for i = 0 : step : th
        % Se TH1 for negativo, faça
        if (TH(1) < 0)
            if (th1 > TH(1))
                th1 = th1 - step;
            else
                th1 = TH(1);
            end
            % Caso contrário:
        else
            if (th1 < TH(1))
                th1 = th1 + step;
            else
                th1 = TH(1);
            end
        end
        
        % Se TH2 for negativo, faça
        if (TH(2) < 0)
            if (th2 > TH(2))
                th2 = th2 - step;
            else
                th2 = TH(2);
            end
            % Caso contrário:
        else
            if (th2 < TH(2))
                th2 = th2 + step;
            else
                th2 = TH(2);
            end
        end
        
        % Ajustado o manipulador da posição desejada:
        vrep.simxSetJointPosition(clientID,RJ_Arm1, th1, vrep.simx_opmode_blocking);
        vrep.simxSetJointPosition(clientID,RJ_Arm2, th2, vrep.simx_opmode_blocking);
    end
else
    disp('Failed connecting to remote API server');
end

vrep.simxAddStatusbarMessage(clientID,'Goodbye V-REP!',vrep.simx_opmode_oneshot);
vrep.simxFinish(clientID);
vrep.delete();
disp('Program ended');