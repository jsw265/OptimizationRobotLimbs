function df_torquedth = df_torquedthFunc(th, lengths, rb)

nJ = length(th);

switch nJ
    case 1
        df_torquedth = df_torquedthFunc1(th, lengths, rb);
    case 2
        df_torquedth = df_torquedthFunc2(th, lengths, rb);
    case 3
        df_torquedth = df_torquedthFunc3(th, lengths, rb);
    case 4
        df_torquedth = df_torquedthFunc4(th, lengths, rb);
    case 5
        df_torquedth = df_torquedthFunc5(th, lengths, rb);
    case 6
        df_torquedth = df_torquedthFunc6(th, lengths, rb);
    otherwise
        df_torquedth = [];
end