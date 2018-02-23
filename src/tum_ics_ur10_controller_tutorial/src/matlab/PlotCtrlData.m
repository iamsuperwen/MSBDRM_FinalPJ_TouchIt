function PlotCtrlData( name, level, fig)
%PLOTCTRLDATA Summary of this function goes here
%   Detailed explanation goes here

y=load(name);

x1=y(:,1:4);
%x2=rad2deg(y(:,5:end));
x2=y(:,5:end);

x2ms=[x1 x2];

x=x2ms(1:4:end,:);



% close all

time_idx=4;
joint_idx=5;
jointp_idx=11;
jointpp_idx=17;


jointd_idx=23;
jointdp_idx=29;

djoint_idx=35;
dpjoint_idx=41;

xef_idx=47;
xd_idx=53;

xefp_idx=59;
xdp_idx=65;

Dxef_idx=71;

torque_idx=77;

detJ_idx=83;

trace_idx=84;
omega_idx=85;

t=x(:,4)-x(1,4);


pos=[-1854  70  1800  1001];
pos=[0  70  1800  1001];

j=fig;

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        %         plot(t,x(:,joint_idx+(i)),'b -',t,x(:,jointd_idx+i),'r -',t,x(:,torque_idx+i), 'm -')
        plot(t,x(:,joint_idx+(i)),'b -')
        ns=sprintf('joint position %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('rad')
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        %         plot(t,x(:,joint_idx+(i)),'b -',t,x(:,jointd_idx+i),'r -',t,x(:,torque_idx+i), 'm -')
        plot(t,x(:,jointp_idx+(i)),'b -')
        ns=sprintf('joint velocity %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('rad/s')
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        plot(t,x(:,xef_idx+(i)),'b -',t,x(:,xd_idx+i),'r -')
        
        ns=sprintf('Cartesian position %d',i+1);
        title(ns)
        xlabel('time [s]')
        if i<3
          ylabel('m')
        else
          ylabel('rad')
        end
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        plot(t,x(:,xefp_idx+(i)),'b -',t,x(:,xdp_idx+i),'r -')
        
        ns=sprintf('Cartesian velocity %d',i+1);
        title(ns)
        xlabel('time [s]')
        if i<3
          ylabel('m/s')
        else
          ylabel('rad/s')
        end
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        plot(t,x(:,Dxef_idx+(i)),'b -')
        ns=sprintf('Cartesian Error %d',i+1);
        title(ns)
        xlabel('time [s]')
        if i<3
          ylabel('m')
        else
          ylabel('rad')
        end
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        plot(t,x(:,torque_idx+(i)),'b -')
        ns=sprintf('Torque %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('Nm')
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    
        plot(t,x(:,detJ_idx),'b -')
        ns=sprintf('Determinant of Jacobian');
        title(ns)
        xlabel('time [s]')
        ylabel(' ')
        grid on
   
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    
        plot(t,x(:,trace_idx),'b -')
        ns=sprintf('trace %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('deg')
        grid on
   
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:2
        subplot(3,2,i+1);
        plot(t,x(:,omega_idx+(i)),'b -')
        ns=sprintf('omega %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('rad')
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    for i=0:5
        subplot(3,2,i+1);
        plot(t,x(:,djoint_idx+(i)),'b -')
        ns=sprintf('Delta X %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('rad')
        grid on
    end
    j=j+1;
end

if(level>0)
    f{j}= figure(j);
    [row,col] = size(x);
    x_temp = [1:1:row];
    % set(f{j}, 'Position', [0 0 1900 1000]);
    set(f{j}, 'Position', pos);
    
    
        plot(x_temp,x(:,time_idx),'b -')
        ns=sprintf('time %d',i+1);
        title(ns)
        xlabel('time [s]')
        ylabel('deg')
        grid on
   
    j=j+1;
end



%cut=100;
%
%if(level>1)
%    f{j}= figure(j);
%    % set(f{j}, 'Position', [0 0 1900 1000]);
%    set(f{j}, 'Position', pos);
%    for i=0:5
%        subplot(6,1,i+1);plot(t(end-cut:end),x(end-cut:end,djoint_idx+(i)),'b -')
%        ns=sprintf('joint Error Position%d',i+1);
%        title(ns)
%        xlabel('time [s]')
%        ylabel('deg')
%        grid on
%    end
%    j=j+1;
%    
%end
%
%if(level>2)
%    f{j}= figure(j);
%    % set(f{j}, 'Position', [0 0 1900 1000]);
%    set(f{j}, 'Position', pos);
%    for i=0:5
%        subplot(6,1,i+1);plot(t,x(:,dpjoint_idx+(i)),'b -')
%        ns=sprintf('joint Error Velocity %d',i+1);
%        title(ns)
%        xlabel('time [s]')
%        ylabel('deg/s')
%        grid on
%    end
%    j=j+1;
%end





end

