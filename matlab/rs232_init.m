function s=rs232_init
if nargout < 1 
    error('syntax error')
else    
    s=serial('COM10','BaudRate',115200);
%     s=serial('COM9','BaudRate',9600);
 
 %--- used for receive 1 byte each time and run callback function

%     s.BytesAvailableFcnMode='byte';
%     s.BytesAvailableFcnCount=1;
%     s.InputBufferSize=1;

 %--- used for receive a string each time, -----------------
    s.InputBufferSize=100;
    s.BytesAvailableFcnMode='Terminator';
    s.Terminator= 'CR/LF';
    
    %s.BytesAvailableFcn=@rs232_callback_3value; %ham su dung de ve 3 so lieu theo thoi gian
    s.BytesAvailableFcn=@rs232_callback;   %ham su dung de ve 1 so lieu theo thoi gian

    fopen(s);
    status='port: open success'       %bat dau dem thoi gian
    s
    status='done and begin to tranfer'
    
end




