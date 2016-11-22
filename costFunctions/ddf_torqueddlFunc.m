function ddf_torqueddl = ddf_torqueddlFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        ddf_torqueddl = ddf_torqueddlFunc1(th, lengths, rb);
    case 2
        ddf_torqueddl = ddf_torqueddlFunc2(th, lengths, rb);
    case 3
        ddf_torqueddl = ddf_torqueddlFunc3(th, lengths, rb);
    case 4
        ddf_torqueddl = ddf_torqueddlFunc4(th, lengths, rb);
    case 5
        ddf_torqueddl = ddf_torqueddlFunc5(th, lengths, rb);
    case 6
        ddf_torqueddl = ddf_torqueddlFunc6(th, lengths, rb);
    otherwise
        ddf_torqueddl = [];
end