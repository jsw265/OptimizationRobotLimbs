function ddfddeffOffset = ddfddeffOffsetFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        ddfddeffOffset = ddfddeffOffsetFunc1(th, lengths, rb, effOffset, Td);
    case 2
        ddfddeffOffset = ddfddeffOffsetFunc2(th, lengths, rb, effOffset, Td);
    case 3
        ddfddeffOffset = ddfddeffOffsetFunc3(th, lengths, rb, effOffset, Td);
    case 4
        ddfddeffOffset = ddfddeffOffsetFunc4(th, lengths, rb,effOffset,  Td);
    case 5
        ddfddeffOffset = ddfddeffOffsetFunc5(th, lengths, rb,effOffset,  Td);
    case 6
        ddfddeffOffset = ddfddeffOffsetFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        ddfddeffOffset = [];
end