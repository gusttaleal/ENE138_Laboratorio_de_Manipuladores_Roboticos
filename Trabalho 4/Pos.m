%##########################################################################
%#               UNIVERSIDADE FEDERAL DE JUIZ DE FORA                     #
%#              GUSTAVO LEAL SILVA E SOUZA - 201469055B                   #
%##########################################################################
function P = Pos(TH , L1 , L2)
    X = L1 * cos( TH(1) ) + L2 * cos( TH(1) + TH(2) );
    Y = L1 * sin( TH(1) ) + L2 * sin( TH(1) + TH(2) );
    P = [ X ; Y ];
end

