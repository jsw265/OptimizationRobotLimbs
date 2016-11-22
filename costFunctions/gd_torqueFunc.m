function dtau = gd_torqueFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        dtau = gd_torqueFunc1(th, lengths, rb);
    case 2
        dtau = gd_torqueFunc2(th, lengths, rb);
    case 3
        dtau = gd_torqueFunc3(th, lengths, rb);
    case 4
        dtau = gd_torqueFunc4(th, lengths, rb);
    case 5
        dtau = gd_torqueFunc5(th, lengths, rb);
    case 6
        dtau = gd_torqueFunc6(th, lengths, rb);
    otherwise
        dtau = [];
end