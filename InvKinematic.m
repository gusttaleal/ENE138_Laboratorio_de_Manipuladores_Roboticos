function [TH1u , TH2u , TH1d , TH2d] = InvKinematic(X, Y, L1, L2, C)
    TH1u = zeros(1,C);  TH2u = zeros(1,C);
    TH1d = zeros(1,C);  TH2d = zeros(1,C);

    Cth = ((X.^2 + Y.^2) - (L1^2 + L2^2)) / (2 * L1 * L2);

    for i = 1:3
        if (abs(Cth(i)) < 1)
            Sth = sqrt(1 - Cth(i)^2);

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
end

