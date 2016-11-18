function define_protocol
global CN_1
global CN_2
global CN_3
global CN_4
global CN_5
global CN_6
global CN_7
global CN_8
global CN_9
global CN_10
CN_1='~';
CN_2='!';
CN_3='@';
CN_4='#';
CN_5='$';
CN_6='%';
CN_7='^';
CN_8='*';
CN_9='(';
CN_10=')';



global DISPLAY_ON  
global DISPLAY_OFF 
DISPLAY_ON ='e'
DISPLAY_OFF ='f'

global RUN      
global STOP      
RUN ='c'
STOP='d'
global SET_POSITION
SET_POSITION='SP';

%state flags indicates state of system
global flag_TEST_MOTOR
global flag_RUN
global flag_DISPLAY
global flag_SET_PARAMETER
flag_SET_PARAMETER=0;
flag_DISPLAY=0;
flag_TEST_MOTOR=0;
flag_RUN=0;

%function flags 
global flag_SCAN_PARAMETER
flag_SCAN_PARAMETER=0;


global serial_data
serial_data={[],[],[],[],[],[],[],[],[],[]};

global display_color
display_color={[1 0 0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[0,0,0],[0.7,0.7,0],[0.7,0.2,0],[0,0.8,0.2]};

global display_CN_enable
display_CN_enable=zeros(1,10);

global plot_figure
plot_figure=zeros(1,10);

global axes_channel
axes_channel={[],[],[],[],[],[],[],[],[],[]};

end



