function ddf_torqueddth = ddf_torqueddthFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        ddf_torqueddth = ddf_torqueddthFunc1(th, lengths, rb);
    case 2
        ddf_torqueddth = ddf_torqueddthFunc2(th, lengths, rb);
    case 3
        ddf_torqueddth = ddf_torqueddthFunc3(th, lengths, rb);
    case 4
        ddf_torqueddth = ddf_torqueddthFunc4(th, lengths, rb);
    case 5
        ddf_torqueddth = ddf_torqueddthFunc5(th, lengths, rb);
    case 6
        ddf_torqueddth = ddf_torqueddthFunc6(th, lengths, rb);
    otherwise
        ddf_torqueddth = [];
end