function tau = g_torqueFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        tau = g_torqueFunc1(th, lengths, rb);
    case 2
        tau = g_torqueFunc2(th, lengths, rb);
    case 3
        tau = g_torqueFunc3(th, lengths, rb);
    case 4
        tau = g_torqueFunc4(th, lengths, rb);
    case 5
        tau = g_torqueFunc5(th, lengths, rb);
    case 6
        tau = g_torqueFunc6(th, lengths, rb);
    otherwise
        tau = [];
end