%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
function [TH1u , TH2u , TH1d , TH2d] = InvKinematic(X, Y, L1, L2, C)
    TH1u = zeros(1,C);  TH2u = zeros(1,C);
    TH1d = zeros(1,C);  TH2d = zeros(1,C);

    Cth = ( (X.^2 + Y.^2) - (L1^2 + L2^2) ) / (2 * L1 * L2);

    for i = 1:length(X)
        if (abs(Cth(i)) < 1)
            Sth = sqrt( 1 - Cth(i)^2 );

            K1 = L1 + (L2 * Cth(i));
            K2 = L2 * Sth;

            R = sqrt(K1^2 + K2^2);

            TH1u(i) = atan2( Y(i) / R , X(i) / R) - atan2( K2, K1);
            TH2u(i) = atan2( Sth , Cth(i) );

            TH1d(i) = atan2( Y(i) / R , X(i) / R) - atan2( -K2, K1);
            TH2d(i) = atan2( -Sth , Cth(i) );

        else
            fprintf('Ponto %d fora do alcance\n', i);
            TH1u(i) = 0;  TH2u(i) = 0;
            TH1d(i) = 0;  TH2d(i) = 0;
        end
    end

    TH1u = mod( TH1u , 2 * pi );
    if(TH1u > pi)
        TH1u = TH1u - ( 2 * pi );
    end

    TH2u = mod( TH2u , 2 * pi );
    if(TH2u > pi)
        TH2u = TH2u - ( 2 * pi );
    end

    TH1d = mod( TH1d , 2 * pi );
    if(TH1d > pi)
        TH1d = TH1d - ( 2 * pi );
    end

    TH2d = mod( TH2d , 2 * pi );
    if(TH2d > pi)
        TH2d = TH2d - ( 2 * pi );
    end
end

