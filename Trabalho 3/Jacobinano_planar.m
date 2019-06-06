function [ J ] = Jacobinano_planar( TH, l1, l2 )
% entrega o cálculo do Jacobiano do manipulador duplo planar
% dados os ângulos theta1, theta2, e o comprimento  dos links L1 e L2
J = [ - l2*sin(TH(1) + TH(2)) - l1*sin(TH(1)), -l2*sin(TH(1) + TH(2))
        l2*cos(TH(1) + TH(2)) + l1*cos(TH(1)),  l2*cos(TH(1) + TH(2))];

end

