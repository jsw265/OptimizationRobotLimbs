function g_st = fkFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        g_st = fkFunc1(th, lengths, rb);
    case 2
        g_st = fkFunc2(th, lengths, rb);
    case 3
        g_st = fkFunc3(th, lengths, rb);
    case 4
        g_st = fkFunc4(th, lengths, rb);
    case 5
        g_st = fkFunc5(th, lengths, rb);
    case 6
        g_st = fkFunc6(th, lengths, rb);
    otherwise
        g_st = [];
end