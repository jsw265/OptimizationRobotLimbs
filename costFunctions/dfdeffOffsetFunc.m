function dfdeffOffset = dfdeffOffsetFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        dfdeffOffset = dfdeffOffsetFunc1(th, lengths, rb, effOffset, Td);
    case 2
        dfdeffOffset = dfdeffOffsetFunc2(th, lengths, rb, effOffset, Td);
    case 3
        dfdeffOffset = dfdeffOffsetFunc3(th, lengths, rb, effOffset, Td);
    case 4
        dfdeffOffset = dfdeffOffsetFunc4(th, lengths, rb,effOffset,  Td);
    case 5
        dfdeffOffset = dfdeffOffsetFunc5(th, lengths, rb,effOffset,  Td);
    case 6
        dfdeffOffset = dfdeffOffsetFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        dfdeffOffset = [];
end