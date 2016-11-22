function f_tau = f_torqueFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        f_tau = f_torqueFunc1(th, lengths, rb);
    case 2
        f_tau = f_torqueFunc2(th, lengths, rb);
    case 3
        f_tau = f_torqueFunc3(th, lengths, rb);
    case 4
        f_tau = f_torqueFunc4(th, lengths, rb);
    case 5
        f_tau = f_torqueFunc5(th, lengths, rb);
    case 6
        f_tau = f_torqueFunc6(th, lengths, rb);
    otherwise
        f_tau = [];
end