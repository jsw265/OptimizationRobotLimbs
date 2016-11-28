function ddfddl = ddfddlFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        ddfddl = ddfddlFunc1(th, lengths, rb, effOffset, Td);
    case 2
        ddfddl = ddfddlFunc2(th, lengths, rb, effOffset, Td);
    case 3
        ddfddl = ddfddlFunc3(th, lengths, rb, effOffset, Td);
    case 4
        ddfddl = ddfddlFunc4(th, lengths, rb,effOffset,  Td);
    case 5
        ddfddl = ddfddlFunc5(th, lengths, rb,effOffset,  Td);
    case 6
        ddfddl = ddfddlFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        ddfddl = [];
end