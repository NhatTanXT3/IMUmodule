function rs232_callback(s,BytesAvailable)
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

% global axes_test

global flag_save_and_plot
global display_color
global display_CN_enable
global serial_data

flag_data_update=zeros(1,10);
data_update=zeros(10,2);

global pre_data_update
global pre_time_display
global pre_clear_display %keep the display speed alway fast
global first_time_display
if (isempty(first_time_display))
    tic; %start timer
    pre_time_display=zeros(1,10);
    pre_data_update=zeros(10,2);
    pre_clear_display=zeros(1,10);
end
first_time_display =1;

current_time=toc;

str_data=fscanf(s)

length_str_data=length(str_data);
str_index=1;
flag_error=0;

%===========check error===============
if(length_str_data==0)
    flag_error=1;
end

if (~flag_error)
    if(str_data(1)==CN_1)
        for i=1:(length_str_data-1)
            switch(str_data(i))
                case CN_1
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(1,:)=[current_time,num];
                        flag_data_update(1)=1;
                    end
                case CN_2
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(2,:)=[current_time,num];
                        flag_data_update(2)=1;
                    end
                case CN_3
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(3,:)=[current_time,num];
                        flag_data_update(3)=1;
                    end
                case CN_4
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(4,:)=[current_time,num];
                        flag_data_update(4)=1;
                    end
                case CN_5
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(5,:)=[current_time,num];
                        flag_data_update(5)=1;
                    end
                case CN_6
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(6,:)=[current_time,num];
                        flag_data_update(6)=1;
                    end
                case CN_7
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(7,:)=[current_time,num];
                        flag_data_update(7)=1;
                    end
                case CN_8
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(8,:)=[current_time,num];
                        flag_data_update(8)=1;
                    end
                case CN_9
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(9,:)=[current_time,num];
                        flag_data_update(9)=1;
                    end
                case CN_10
                    num=aton(str_data((i+1):length(str_data)));
                    if(~isnan(num))
                        data_update(10,:)=[current_time,num];
                        flag_data_update(10)=1;
                    end 
            end
        end
    else
        str_data
    end
end

if(flag_save_and_plot)
    global axes_data
    global axes_channel
    global plot_figure
    
    for i=1:10
        if(flag_data_update(i)==1)
            flag_data_update(i)=0;
            serial_data{i}=[serial_data{i};data_update(i,:)];            
              if(display_CN_enable(i))
                if((current_time-pre_time_display(i))>0.1)
                    pre_time_display(i)=current_time;
                    
                    plot(axes_channel{plot_figure(i)},[pre_data_update(i,1),data_update(i,1)],[pre_data_update(i,2),data_update(i,2)],'Color',display_color{i});
                    
                    pre_data_update(i,:)=data_update(i,:);
                    axis(axes_channel{plot_figure(i)},[(current_time-10) (current_time+5)  0 300])
                    axis(axes_channel{plot_figure(i)},'autoy');
                    
                    if((current_time-pre_clear_display(i))>20)
                        pre_clear_display(i)=current_time;
                        cla(axes_channel{plot_figure(i)});
                        %                         cla(axes_test);
                    end
                end
              end
            
              
        end
    end
end

end

