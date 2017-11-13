%data = data1;
data = importLine();
%data = Experiment1;

dt= 0.1;        %time per measure
r = (55/2)*(10^-3);  %in metres
b = 60*(10^-3);         %0.12/2 meters

pure_sensor_track = [0,0];
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

for i = 2:(length(data))

    currstate = data(i,:);
    prevstate = data(i-1,:);

    sr = currstate(3);
    sl = currstate(4);
    dsl = ((sl-p_sl))*r;
    dsr = ((sr-p_sr))*r; 

    c_p = F(p_p,dsr,dsl,b);
    
    point = [c_p(1),c_p(2)];
    pure_sensor_track = [pure_sensor_track; point];
    pure_sensor_track_error = [pure_sensor_track_error, c_p(3)-currstate(2)];
    log = [log; (c_p)];
    
    p_p = c_p;
    p_sr = sr;
    p_sl = sl;

end


plot(pure_sensor_track(:,1),pure_sensor_track(:,2));