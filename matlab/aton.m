
%
% Get a number from a string
% Return a number 
%
function kq=aton(str)
num_str='';
if(ischar(str))
    StrLength=length(str);
    Str_index=1;
    
    flag_sign=0; %check the apperance of sign character: '+','-' or '' in the first character of the string
    flag_dot=0; %check the apperance of dot character: '.'
    
    if((str(Str_index)=='+')|(str(Str_index)=='-'))
        num_str(1)=str(1);
        Str_index=2;
    end

    for i=Str_index:StrLength
        char_num=str(i);
        if(isNumASCII(str(i))|(str(i)=='.'))
            num_str(i)=char_num;
        else
            break;
        end
    end
    kq=str2double(num_str);
else
    kq=nan;
end %end of if
end %end of this function

%check whether the character is a number of not
function kq = isNumASCII(char)
if((char>=48)&(char<=57))
    kq=1;
else
    kq=0;
end
end