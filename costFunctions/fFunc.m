function f = fFunc(th, lengths, rb, Td)

nJ = length(th);

switch nJ
    case 1
        f = fFunc1(th, lengths, rb, Td);
    case 2
        f = fFunc2(th, lengths, rb, Td);
    case 3
        f = fFunc3(th, lengths, rb, Td);
    case 4
        f = fFunc4(th, lengths, rb, Td);
    case 5
        f = fFunc5(th, lengths, rb, Td);
    case 6
        f = fFunc6(th, lengths, rb, Td);
    otherwise
        f = [];
end