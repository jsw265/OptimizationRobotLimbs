function ddfddth = ddfddthFunc(th, lengths, rb, Td)

nJ = length(th);

switch nJ
    case 1
        ddfddth = ddfddthFunc1(th, lengths, rb, Td);
    case 2
        ddfddth = ddfddthFunc2(th, lengths, rb, Td);
    case 3
        ddfddth = ddfddthFunc3(th, lengths, rb, Td);
    case 4
        ddfddth = ddfddthFunc4(th, lengths, rb, Td);
    case 5
        ddfddth = ddfddthFunc5(th, lengths, rb, Td);
    case 6
        ddfddth = ddfddthFunc6(th, lengths, rb, Td);
    otherwise
        ddfddth = [];
end