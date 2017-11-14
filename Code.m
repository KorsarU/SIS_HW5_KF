data = importfile('data.csv', 1, 301);
%data = importLine();
%data = Sin;
%data = SinCopy;

dt= 0.1;    %time per measure
r = (55/2); 
b = 60;      

pure_sensor_track = [0,0];
kalman_sensor_track = [0,0];
pure_sensor_track_error = [];
log = [0,0,0];
%x;y;sr;sl;th
q = [0;0;0;0;0];
%x;y;th
p_p = [0,0,0];
sr= 0;
sl= 0;
kr = 0;
kl = 0;
p_sr = data(1, 3);
p_sl = data(1, 4);

Q11=1e-4*1.00;
Q22=1e-4*1.00;
Q33=7.62e-5*1.00;
Q=diag([Q11,Q22,Q33]);

P_initial=Q;

traceP_initial=trace(P_initial);
traceResetValue=traceP_initial*0.5;
P=P_initial;

azimuthCov=0.048^2;
inclinCov=0.006^2;
singleSensorCov=[azimuthCov inclinCov];
R=[];
R=diag(singleSensorCov);
varIn=1e-4;


for i = 2:(length(data))

    currstate = data(i,:);
    prevstate = data(i-1,:);

    sr = currstate(3);
    sl = currstate(4);
    dsl = (sl-p_sl)*r;
    dsr = (sr-p_sr)*r; ds = (dsr+dsl)/2; dth = (dsr-dsl)/b;
    
    c_p = F(p_p,dsr,dsl,b);
    point = [c_p(1),c_p(2)];
    pure_sensor_track = [pure_sensor_track; point];
    
    p_p = c_p;
    p_sr = p_sr+dsr/r;
    p_sl = p_sl+dsl/r;
    
    x = c_p(1);
    y = c_p(2);
    th = c_p(3);
    
    xk = [x;y;th];
    uk = [dsl; dsr];
    
    fx = x+ds*cos(th+dth/2);
    fy = y+ds*sin(th+dth/2);
    fth= th+dth;
    
    fX = [fx, fy,fth];
    
    A = [...
      1,0, -ds*sin(th+dth/2);...
      0,1, ds*cos(th+dth/2);...
      0,0,1 ...
    ];

    B = [...
      (1/2)*cos(th+dth/2) + ds/(2*b)*sin(th+dth/2),  (1/2)*cos(th + dth/2) - ds/(2*b)*sin(th+dth/2);...
      (1/2)*cos(th+dth/2) - ds/(2*b)*sin(th+dth/2),  (1/2)*cos(th + dth/2) + ds/(2*b)*sin(th+dth/2);...
      -1/b, 1/b ...
    ];
    
    Ik=eye(2,2);
    P=A*P*A'+varIn*B*Ik*B'+Q;

    H = [...
        0,0,-1;...
        0,0, 1;...
        ];
    
    K=P*H'*(inv(H*P*H'+R));
    
    z = [currstate(2);0];
    
    innov=z-th;

    corr=K*innov;
    c_p = c_p+corr';      
   
    point = [c_p(1),c_p(2)];
    kalman_sensor_track = [kalman_sensor_track; point];
    
    log = [log; (c_p)];
    


end


subplot(2,2,1)
plot(pure_sensor_track(:,1),pure_sensor_track(:,2)); title('without filter');
subplot(2,2,2)
plot(kalman_sensor_track(:,1),kalman_sensor_track(:,2)); title('with Kalman');