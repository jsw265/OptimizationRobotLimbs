function ddfddl = ddfddlFunc(th, lengths, rb, Td)

nJ = length(th);

switch nJ
    case 1
        ddfddl = ddfddlFunc1(th, lengths, rb, Td);
    case 2
        ddfddl = ddfddlFunc2(th, lengths, rb, Td);
    case 3
        ddfddl = ddfddlFunc3(th, lengths, rb, Td);
    case 4
        ddfddl = ddfddlFunc4(th, lengths, rb, Td);
    case 5
        ddfddl = ddfddlFunc5(th, lengths, rb, Td);
    case 6
        ddfddl = ddfddlFunc6(th, lengths, rb, Td);
    otherwise
        ddfddl = [];
end