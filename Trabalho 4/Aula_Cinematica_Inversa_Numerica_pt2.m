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
    
    % Tamanho dos links
    L1 = 0.1;   L2 = 0.1;
    
    % �ngulos iniciais das juntas
    TH = [ 0 ; 0 ];
    
    % Fun��o para aquisi��o da posi��o atual do manipulador
    P = Pos(TH, L1, L2);
    
    % Conjunto de pontos objetivo
    G = [ 0.05 ; 0.07 ];
    
    % Erro
    E = G - P;
    
    % Erro quadratico m�dio
    EQM = sqrt( E' * E );
    
    % Constantes de treinamento
    Eta = 1e-3;
    Csi = 1e-6;
    
    % Ganhos da matriz K
    K = [ 1e3 , 0 ; 0 , 1e3 ];
    
    % Por��o de tempo
    dTime = 0.01;
    
    while( EQM > Csi )
        % Inicia o contador
        tic
        
        % Erro
        E = G - P;
        
        % Erro quadratico m�dio
        EQM = sqrt( E' * E );
        
        % Aquisi��o do Jacobiano da fun��o f(Th1 , Th2)
        J = Jacobinano_planar(TH, L1, L2);
        
        %
        dq = J' * ( K * E );
        
        % Integral de Euler
        TH = TH + ( dq * dTime );
        
        % Tempo atual
        t = toc;
        
        % Tempo de cada a��o
        pause(dTime - t);
        
        % Fun��o para aquisi��o da posi��o atual do manipulador
        P = Pos(TH, L1, L2);
        
    end
    
    rad2deg(TH)
    
    % Corre��o do �ngulo TH1 para o intervalo [ 0� , 360� ]
    TH(1) = mod( TH(1) , 2 * pi );
    if(TH(1) > pi)
        TH(1) = TH(1) - ( 2 * pi );
    end
    
    % Corre��o do �ngulo TH2 para o intervalo [ 0� , 360� ]
    TH(2) = mod( TH(2) , 2 * pi );
    if(TH(2) > pi)
        TH(2) = TH(2) - ( 2 * pi );
    end
    
    rad2deg(TH)
    
    % �ngulo m�ximo
    th = max( abs(TH(1)) , abs(TH(2)));
    
    % Passo
    step = deg2rad(1);
    
    % �ngulos Iniciais
    th1 = 0;
    th2 = 0;
    
    for i = 0 : step : th
        % Se TH1 for negativo, fa�a
        if (TH(1) < 0)
            if (th1 > TH(1))
                th1 = th1 - step;
            else
                th1 = TH(1);
            end
            % Caso contr�rio:
        else
            if (th1 < TH(1))
                th1 = th1 + step;
            else
                th1 = TH(1);
            end
        end
        
        % Se TH2 for negativo, fa�a
        if (TH(2) < 0)
            if (th2 > TH(2))
                th2 = th2 - step;
            else
                th2 = TH(2);
            end
            % Caso contr�rio:
        else
            if (th2 < TH(2))
                th2 = th2 + step;
            else
                th2 = TH(2);
            end
        end
        
        % Ajustado o manipulador da posi��o desejada:
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