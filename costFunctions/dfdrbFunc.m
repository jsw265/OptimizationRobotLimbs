function dfdrb = dfdrbFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        dfdrb = dfdrbFunc1(th, lengths, rb, effOffset, Td);
    case 2
        dfdrb = dfdrbFunc2(th, lengths, rb, effOffset, Td);
    case 3
        dfdrb = dfdrbFunc3(th, lengths, rb, effOffset, Td);
    case 4
        dfdrb = dfdrbFunc4(th, lengths, rb,effOffset,  Td);
    case 5
        dfdrb = dfdrbFunc5(th, lengths, rb,effOffset,  Td);
    case 6
        dfdrb = dfdrbFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        dfdrb = [];
end