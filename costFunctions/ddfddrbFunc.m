function ddfddrb = ddfddrbFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        ddfddrb = ddfddlFunc1(th, lengths, rb, effOffset, Td);
    case 2
        ddfddrb = ddfddlFunc2(th, lengths, rb, effOffset, Td);
    case 3
        ddfddrb = ddfddlFunc3(th, lengths, rb, effOffset, Td);
    case 4
        ddfddrb = ddfddlFunc4(th, lengths, rb,effOffset,  Td);
    case 5
        ddfddrb = ddfddlFunc5(th, lengths, rb,effOffset,  Td);
    case 6
        ddfddrb = ddfddlFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        ddfddrb = [];
end