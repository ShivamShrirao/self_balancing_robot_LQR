global A = csvread('sensor_data.csv');  #do not change this line

################################################
#######Declare your global variables here#######
################################################


function read_accel(axl,axh,ayl,ayh,azl,azh)  
  
  #################################################
  ####### Write a code here to combine the ########
  #### HIGH and LOW values from ACCELEROMETER #####
  #################################################
  scaling_factor=16384;
  ax=double(typecast([uint8(axl), uint8(axh)], 'int16'))/scaling_factor;
  ay=double(typecast([uint8(ayl), uint8(ayh)], 'int16'))/scaling_factor;
  az=double(typecast([uint8(azl), uint8(azh)], 'int16'))/scaling_factor;
  ####################################################
  # Call function lowpassfilter(ax,ay,az,f_cut) here #
  ####################################################
  lowpassfilter(ax,ay,az,5);

endfunction

function read_gyro(gxl,gxh,gyl,gyh,gzl,gzh)
  
  #################################################
  ####### Write a code here to combine the ########
  ###### HIGH and LOW values from GYROSCOPE #######
  #################################################
  scaling_factor=131;
  gx=double(typecast([uint8(gxl), uint8(gxh)], 'int16'))/scaling_factor;
  gy=double(typecast([uint8(gyl), uint8(gyh)], 'int16'))/scaling_factor;
  gz=double(typecast([uint8(gzl), uint8(gzh)], 'int16'))/scaling_factor;
  #####################################################
  # Call function highpassfilter(gx,gy,gz,f_cut) here #
  #####################################################;
  highpassfilter(gx,gy,gz,5);

endfunction



function lowpassfilter(ax,ay,az,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  global lowx lowy lowz glax glay glaz;
  lowx=(1-alpha)*ax + alpha*lowx;
  glax=lowx;
  lowy=(1-alpha)*ay + alpha*lowy;
  glay=lowy;
  lowz=(1-alpha)*az + alpha*lowz;
  glaz=lowz;
endfunction



function highpassfilter(gx,gy,gz,f_cut)
  dT = 0.01;  #time in seconds
  Tau= 1/(2*pi*f_cut);
  alpha = Tau/(Tau+dT);                #do not change this line
  
  ################################################
  ##############Write your code here##############
  ################################################
  global highx highy highz ogx ogy ogz glgx glgy glgz;
  highx=(1-alpha)*highx + (1-alpha)*(gx-ogx);
  ogx=gx;
  glgx=highx;
  highy=(1-alpha)*highy + (1-alpha)*(gy-ogy);
  ogy=gy;
  glgy=highy;
  highz=(1-alpha)*highz + (1-alpha)*(gz-ogz);
  ogz=gz;
  glgz=highz;

endfunction

function comp_filter_pitch(ax,ay,az,gx,gy,gz)

  ##############################################
  ####### Write a code here to calculate  ######
  ####### PITCH using complementry filter ######
  ##############################################
  global pitch_angle roll_angle;
  alpha=0.03;
  dt=0.01;
  acc_angle=atan2d(ay,abs(az));
  gyro_angle=(-1*gx*dt) + pitch_angle;
  pitch_angle=(1-alpha)*gyro_angle + alpha*acc_angle;

endfunction 

function comp_filter_roll(ax,ay,az,gx,gy,gz)

  ##############################################
  ####### Write a code here to calculate #######
  ####### ROLL using complementry filter #######
  ##############################################
  global pitch_angle roll_angle;
  alpha=0.03;
  dt=0.01;
  acc_angle=atan2d(ax,abs(az));
  gyro_angle=(-1*gy*dt) + roll_angle;
  roll_angle=(1-alpha)*gyro_angle + alpha*acc_angle;

endfunction 

function execute_code
  global A glax glay glaz glgx glgy glgz pitch_angle roll_angle B;
  B=[];
  pitch_angle=0;
  roll_angle=0;
  for n = 1:rows(A)                    #do not change this line

    ###############################################
    ####### Write a code here to calculate  #######
    ####### PITCH using complementry filter #######
    ###############################################
    if n==1
      global lowx lowy lowz highx highy highz ogx ogy ogz;
      lowx=lowy=lowz=highx=highy=highz=ogx=ogy=ogz=0;
    endif
    hld=num2cell(A(n,1:6));
    [axh,axl,ayh,ayl,azh,azl]=deal(hld{:});
    read_accel(axl,axh,ayl,ayh,azl,azh);
    hld=num2cell(A(n,7:12));
    [gxh,gxl,gyh,gyl,gzh,gzl]=deal(hld{:});
    read_gyro(gxl,gxh,gyl,gyh,gzl,gzh);
    comp_filter_pitch(glax,glay,glaz,glgx,glgy,glgz);
    comp_filter_roll(glax,glay,glaz,glgx,glgy,glgz);
    B(n,1)=pitch_angle;
    B(n,2)=roll_angle;
  endfor
  csvwrite('output_data.csv',B);        #do not change this line
endfunction


execute_code                           #do not change this line
