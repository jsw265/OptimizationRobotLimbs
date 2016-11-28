function dfdl = dfdlFunc(th, lengths, rb, effOffset, Td)

nJ = length(th);

switch nJ
    case 1
        dfdl = dfdlFunc1(th, lengths, rb, effOffset, Td);
    case 2
        dfdl = dfdlFunc2(th, lengths, rb, effOffset, Td);
    case 3
        dfdl = dfdlFunc3(th, lengths, rb, effOffset, Td);
    case 4
        dfdl = dfdlFunc4(th, lengths, rb, effOffset, Td);
    case 5
        dfdl = dfdlFunc5(th, lengths, rb, effOffset, Td);
    case 6
        dfdl = dfdlFunc6(th, lengths, rb, effOffset, Td);
    otherwise
        dfdl = [];
end