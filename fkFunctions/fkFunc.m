function g_st = fkFunc(th, lengths, rb, effOffset)

nJ = length(th);

switch nJ
    case 1
        g_st = fkFunc1(th, lengths, rb, effOffset);
    case 2
        g_st = fkFunc2(th, lengths, rb, effOffset);
    case 3
        g_st = fkFunc3(th, lengths, rb, effOffset);
    case 4
        g_st = fkFunc4(th, lengths, rb, effOffset);
    case 5
        g_st = fkFunc5(th, lengths, rb, effOffset);
    case 6
        g_st = fkFunc6(th, lengths, rb, effOffset);
    otherwise
        disp('nJ not supported');
        g_st = [];
end