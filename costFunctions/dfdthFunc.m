function dfdth = dfdthFunc(th, lengths, rb, Td)

nJ = length(th);

switch nJ
    case 1
        dfdth = dfdthFunc1(th, lengths, rb, Td);
    case 2
        dfdth = dfdthFunc2(th, lengths, rb, Td);
    case 3
        dfdth = dfdthFunc3(th, lengths, rb, Td);
    case 4
        dfdth = dfdthFunc4(th, lengths, rb, Td);
    case 5
        dfdth = dfdthFunc5(th, lengths, rb, Td);
    case 6
        dfdth = dfdthFunc6(th, lengths, rb, Td);
    otherwise
        dfdth = [];
end