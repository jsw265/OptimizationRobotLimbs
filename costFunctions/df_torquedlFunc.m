function df_torquedl = df_torquedlFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        df_torquedl = df_torquedlFunc1(th, lengths, rb);
    case 2
        df_torquedl = df_torquedlFunc2(th, lengths, rb);
    case 3
        df_torquedl = df_torquedlFunc3(th, lengths, rb);
    case 4
        df_torquedl = df_torquedlFunc4(th, lengths, rb);
    case 5
        df_torquedl = df_torquedlFunc5(th, lengths, rb);
    case 6
        df_torquedl = df_torquedlFunc6(th, lengths, rb);
    otherwise
        df_torquedl = [];
end