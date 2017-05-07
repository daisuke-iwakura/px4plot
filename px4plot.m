% THIS IS ALPHA VERSION. THIS MIGHT CHANGE IN FUTURE VERSION.
%
% -- px4plot(FILEPATH)
% -- px4plot(FILEPATH, '|XXXs')
% -- px4plot(FILEPATH, 'XXXs:YYYs')
% -- px4plot(FILEPATH, 'start+XXXs:end-YYYs')
%     Plot of .px4log binary file logged by px4 autopilot.
%
%     Example :
%       px4plot('01_02_03.px4log');
%     => The function plots major data by reading '01_02_03.px4log'.
%
%       px4plot('01_02_03.px4log', '|20s');
%     => The function plots major data and draws a vertical line at the
%     point t=20 second from the head of log.
%
%       px4plot('01_02_03.px4log', '|20s', '|30s')
%     => Multiple vertical line definition allowed.
%
%       px4plot('01_02_03.px4log', '10s:30s')
%     => Change a range of time-axis to [10 30] in second.
%
%       px4plot('01_02_03.px4log', 'start+10s:end-30s')
%     => Change a range of time-axis based on start and end point.
%
%     Copyright (c) 2017, Daisuke Iwakura
%     All rights reserved.

%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions 
% are met:
%
% 1. Redistributions of source code must retain the above copyright
% notice, this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright
% notice, this list of conditions and the following disclaimer in the
% documentation and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
% A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function px4plot(filename, varargin)
close all;

param = parse_arg(varargin{:});

if ~isstruct(filename)
    log = decode_px4log(filename);
else
    log = filename;
end
    
% Get number of rotors
nRotors = getNRotors(log);

% Get RC Map
RC_MAP.Throttle = getLogData(log, '.parm.RC_MAP_THROTTLE');
RC_MAP.Roll     = getLogData(log, '.parm.RC_MAP_ROLL');
RC_MAP.Pitch    = getLogData(log, '.parm.RC_MAP_PITCH');
RC_MAP.Yaw      = getLogData(log, '.parm.RC_MAP_YAW');

% Time
time = getLogData(log, '.time');
if ~isempty(param.t_end)
    param.t_range(2) = param.t_end;
else    
    param.t_range(2) = time(end) - param.t_end_from;
end
if ~isempty(param.t_start)
    param.t_range(1) = param.t_start;
else
    param.t_range(1) = time(1) + param.t_start_from;
end
    
% ATTITUDE
[ATT t_ATT] = getLogData(log, 'ATT');
ATT.Roll  = ATT.Roll  * 180 / pi;
ATT.Pitch = ATT.Pitch * 180 / pi;
ATT.Yaw   = ATT.Yaw   * 180 / pi;
ATT.RollRate  = ATT.RollRate  * 180 / pi;
ATT.PitchRate = ATT.PitchRate * 180 / pi;
ATT.YawRate   = ATT.YawRate   * 180 / pi;

% ATTITUDE SETPOINT
[ATSP t_ATSP] = getLogData(log, 'ATSP');
ATSP.RollSP  = ATSP.RollSP  * 180 / pi;
ATSP.PitchSP = ATSP.PitchSP * 180 / pi;
ATSP.YawSP   = ATSP.YawSP   * 180 / pi;

% ATTITUDE RATE SET POINT
[ARSP, t_ARSP] = getLogData(log, 'ARSP');
ARSP.RollRateSP  = ARSP.RollRateSP  * 180 / pi;
ARSP.PitchRateSP = ARSP.PitchRateSP * 180 / pi;
ARSP.YawRateSP   = ARSP.YawRateSP   * 180 / pi;

% IMU
[IMU, t_IMU] = getLogData(log, 'IMU');
IMU.GyroX = IMU.GyroX * 180 / pi;
IMU.GyroY = IMU.GyroY * 180 / pi;
IMU.GyroZ = IMU.GyroZ * 180 / pi;

% LOCAL POSITION
[LPOS t_LPOS] = getLogData(log, 'LPOS');

% LOCAL POSITION SETPOINT
[LPSP t_LPSP] = getLogData(log, 'LPSP');
LPSP.Yaw = LPSP.Yaw * 180 / pi;

% GPS POSITION
[GPS t_GPS] = getLogData(log, 'GPS');

% Barometer
[SENS, t_SENS] = getLogData(log, 'SENS');

% ATTITUDE CONTROLS
[ATTC t_ATTC] = getLogData(log, 'ATTC');

% VEHICLE STATUS
[STAT t_STAT] = getLogData(log, 'STAT');

% RC INPUT CHANNELS
[RC t_RC] = getLogData(log, 'RC');

% Servo Pulse Output
[Out0 t_Out0] = getLogData(log, 'OUT0');

% GLOBAL POSITION ESTIMATE
[GPOS t_GPOS] = getLogData(log, 'GPOS');

% GLOBAL POSITION SETPOINT
%[GPSP t_GPSP] = getLogData(log, 'GPSP');
%GPSP.Yaw = GPSP.Yaw * 180 / pi;

% GLOBAL VELOCITY SETPOINT
[GVSP t_GVSP] = getLogData(log, 'GVSP');

% BATTERY
[BATT t_BATT] = getLogData(log, 'BATT');

% TELEMETRY STATUS
[TEL0 t_TEL0] = getLogData(log, 'TEL0');

% ONBOARD POWER SYSTEM
[PWR t_PWR] = getLogData(log, 'PWR');

% LAND DETECTOR
[LAND t_LAND] = getLogData(log, 'LAND');

% SYSTEM LOAD
[LOAD t_LOAD] = getLogData(log, 'LOAD');

% Angular Rate
figure('Name', 'Angular Rate (R:SetPoint B:Controlled G:Raw)', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_IMU, IMU.GyroX, 'g', t_ATT, ATT.RollRate, 'b', t_ARSP, ARSP.RollRateSP, 'r');
figtool(param, 'x Angular vel. [deg/s]');
subplot(3,1,2)
plot(t_IMU, IMU.GyroY, 'g', t_ATT, ATT.PitchRate, 'b', t_ARSP, ARSP.PitchRateSP, 'r');
figtool(param, 'y Angular vel. [deg/s]');
subplot(3,1,3);
plot(t_IMU, IMU.GyroZ, 'g', t_ATT, ATT.YawRate, 'b', t_ARSP, ARSP.YawRateSP, 'r');
figtool(param, 'z Angular vel. [deg/s]');

% Attitude
figure('Name', 'Attitude (R:SetPoint B:Controlled)', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_ATT, ATT.Roll, 'b', t_ATSP, ATSP.RollSP, 'r');
figtool(param, 'Roll [deg]');
subplot(3,1,2)
plot(t_ATT, ATT.Pitch, 'b', t_ATSP, ATSP.PitchSP, 'r');
figtool(param, 'Pitch [deg]');
subplot(3,1,3);
plot(t_ATT, ATT.Yaw, 'b', t_ATSP, ATSP.YawSP, 'r');
%plot(t_ATT, ATT.Yaw, 'b', t_ATSP, ATSP.YawSP, 'r', t_LPSP, LPSP.Yaw, 'k');
figtool(param, 'Yaw [deg]');

% NED Velocity
figure('Name', 'NED Velocity (R:SetPoint B:Controlled G:Raw)', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_GPS, GPS.VelN, 'g', t_LPOS, LPOS.VX, 'b', t_LPSP, LPSP.VX, 'r');
figtool(param, 'North vel. [m/s]');
subplot(3,1,2)
plot(t_GPS, GPS.VelE, 'g', t_LPOS, LPOS.VY, 'b', t_LPSP, LPSP.VY, 'r');
figtool(param, 'East vel. [m/s]');
subplot(3,1,3);
plot(t_GPS, GPS.VelD, 'g', t_LPOS, LPOS.VZ, 'b', t_LPSP, LPSP.VZ, 'r');
figtool(param, 'Doen vel. [m/s]');

% Position
figure('Name', 'Position (R:SetPoint B:Controlled)', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_LPOS, LPOS.X, 'b', t_LPSP, LPSP.X, 'r');
figtool(param, 'North pos. [m]');
subplot(3,1,2)
plot(t_LPOS, LPOS.Y, 'b', t_LPSP, LPSP.Y, 'r');
figtool(param, 'East pos. [m]');
subplot(3,1,3)
plot(t_LPOS, LPOS.Z, 'b', t_LPSP, LPSP.Z, 'r');
figtool(param, 'Down pos. [m]');

% Altitude
figure('Name', 'Altitude', 'NumberTitle', 'off');
BaroAlt2 = SENS.BaroAlt - mean(SENS.BaroAlt) + mean(-LPOS.Z);
hold on
plot(t_SENS, BaroAlt2, 'g');
plot(t_GPS,  GPS.Alt,  'c');
plot(t_LPOS, -LPOS.Z,  'b');
plot(t_LPSP, -LPSP.Z,  'r');
%plot(t_LPOS, LPOS.RAlt, '--k');
legend0('Baro(no bias)', 'GNSS', 'Controlled', 'SetPoint');
figtool(param, 'Altitude [m]');
clear h1 h2 h3 h4 h5

% 3D Position
idx_LPOS_start = findindex(t_LPOS, param.t_range(1));
idx_LPOS_end   = findindex(t_LPOS, param.t_range(2));
figure('Name', '3D Position', 'NumberTitle', 'off');
hold on
plot3( LPOS.Y(idx_LPOS_start:idx_LPOS_end), ...
       LPOS.X(idx_LPOS_start:idx_LPOS_end), ...
      -LPOS.Z(idx_LPOS_start:idx_LPOS_end));
h1 = [];
idx_LPOS_Land = findindex(t_LPOS, t_LAND);
for i = 1 : length(idx_LPOS_Land)
    if idx_LPOS_Land(i) >= 2 && ...
       idx_LPOS_start <= idx_LPOS_Land(i) && ...
       idx_LPOS_Land(i) <= idx_LPOS_end
        h1 = plot3(LPOS.Y(idx_LPOS_Land(i)), ...
                   LPOS.X(idx_LPOS_Land(i)), ...
                   -LPOS.Z(idx_LPOS_Land(i)), '*r');
    end
end
xlabel('East pos. [m]');
ylabel('North pos. [m]');
zlabel('Altitude [m]');
if ~isempty(h1)
    legend0(h1, 'Landed');
end
axis equal
grid on
clear h1

% Accelerometer
figure('Name', 'Accelerometer', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_IMU, IMU.AccX, 'b');
figtool(param, 'x Acceleration [m/s^2]');
subplot(3,1,2)
plot(t_IMU, IMU.AccY, 'b');
figtool(param, 'y Acceleration [m/s^2]');
subplot(3,1,3);
plot(t_IMU, IMU.AccZ, 'b');
figtool(param, 'z Acceleration [m/s^2]');

% Magnetic Field
Mag_Norm = sqrt(IMU.MagX.^2 + IMU.MagY.^2 + IMU.MagZ.^2);

figure('Name', 'Magnetic Field', 'NumberTitle', 'off');
subplot(2,1,1)
plot(t_IMU, IMU.MagX, t_IMU, IMU.MagY, t_IMU, IMU.MagZ);
legend0('X', 'Y', 'Z');
figtool(param, 'Mag. Field [Gauss]');
subplot(2,1,2)
plot(t_IMU, Mag_Norm, 'k');
legend0('Norm (M_X^2+M_Y^2+M_Z^2)^0.5');
figtool(param, 'Mag. Field [Gauss]');

% Barometer
figure('Name', 'Barometer', 'NumberTitle', 'off');
subplot(2,1,1)
plot(t_SENS, SENS.BaroAlt);
figtool(param, 'Altitude [m]');
subplot(2,1,2);
plot(t_SENS, SENS.BaroTemp);
figtool(param, 'Temp. [C]');

% Control Input
figure('Name', 'Control Input', 'NumberTitle', 'off');
subplot(4,1,1);
plot(t_ATTC, ATTC.Roll);
figtool(param, 'Roll');
subplot(4,1,2);
plot(t_ATTC, ATTC.Pitch);
figtool(param, 'Pitch');
subplot(4,1,3);
plot(t_ATTC, ATTC.Yaw);
figtool(param, 'Yaw');
subplot(4,1,4);
plot(t_ATTC, ATTC.Thrust);
figtool(param, 'Thrust');

% Actuator
figure('Name', 'Actuator Output', 'NumberTitle', 'off');
switch nRotors
    case 4
        plot(t_Out0, Out0.Out0, t_Out0, Out0.Out1, t_Out0, Out0.Out2, t_Out0, Out0.Out3);
        legend0('Out0', 'Out1', 'Out2', 'Out3');
    case 6
        plot(t_Out0, Out0.Out0, t_Out0, Out0.Out1, t_Out0, Out0.Out2, t_Out0, Out0.Out3, ...
             t_Out0, Out0.Out4, t_Out0, Out0.Out5);
        legend0('Out0', 'Out1', 'Out2', 'Out3', 'Out4', 'Out5');
    otherwise
        plot(t_Out0, Out0.Out0, t_Out0, Out0.Out1, t_Out0, Out0.Out2, t_Out0, Out0.Out3, ...
             t_Out0, Out0.Out4, t_Out0, Out0.Out5, t_Out0, Out0.Out6, t_Out0, Out0.Out7);
        legend0('Out0', 'Out1', 'Out2', 'Out3', 'Out4', 'Out5', 'Out6', 'Out7');
end

ylim([1000 2000]);
figtool(param, '');

% RC Input
Rc(1).Value = RC.C0;
Rc(2).Value = RC.C1;
Rc(3).Value = RC.C2;
Rc(4).Value = RC.C3;
Rc(5).Value = RC.C4;
Rc(6).Value = RC.C5;
Rc(7).Value = RC.C6;
Rc(8).Value = RC.C7;
Rc(9).Value = RC.C8;
Rc(10).Value = RC.C9;
Rc(11).Value = RC.C10;
Rc(12).Value = RC.C11;
args{1} = t_RC; args{2} = Rc(RC_MAP.Roll).Value;
args{3} = t_RC; args{4} = Rc(RC_MAP.Pitch).Value;
args{5} = t_RC; args{6} = Rc(RC_MAP.Yaw).Value;
args{7} = t_RC; args{8} = Rc(RC_MAP.Throttle).Value;

figure('Name', 'RC Input', 'NumberTitle', 'off');
subplot(3,1,1)
plot(args{:});
legend0('Roll', 'Pitch', 'Yaw', 'Throttle');
figtool(param, '');
subplot(3,1,2)
plot(t_RC, RC.C4, t_RC, RC.C5, t_RC, RC.C6, t_RC, RC.C7);
legend0('Ch4', 'Ch5', 'Ch6', 'Ch7');
figtool(param, '');
subplot(3,1,3)
plot(t_RC, RC.C8, t_RC, RC.C9, t_RC, RC.C10, t_RC, RC.C11);
legend0('Ch8', 'Ch9', 'Ch10', 'Ch11');
figtool(param, '');

% GPS Status
figure('Name', 'GPS Status', 'NumberTitle', 'off');
subplot(4,1,1)
plot(t_GPS, GPS.VelN, t_GPS, GPS.VelE, t_GPS, GPS.VelD);
legend0('North', 'East', 'Down');
figtool(param, 'Velocity [m/s]');
subplot(4,1,2)
plot(t_GPS, GPS.EPH, t_GPS, GPS.EPV)
legend0('Horizontal', 'Vertical');
figtool(param, 'Pos. Accuracy [m]');
subplot(4,1,3);
plot(t_GPS, GPS.nSat, t_GPS, GPS.J, t_GPS, GPS.SNR, t_GPS, GPS.Fix);
legend0('n Sats', 'Jamming Indicator', 'SNR Ave.', 'Fix Type');
figtool(param, '');
subplot(4,1,4);
plot(t_GPS, GPS.N);
figtool(param, 'Noise Level');

% Vehicle Status
figure('Name', 'Vehicle Status', 'NumberTitle', 'off');
subplot(2,1,1)
plot(t_STAT, STAT.MainState, ...
     t_STAT, STAT.NavState,  ...
     t_STAT, STAT.ArmS,      ...
     t_STAT, STAT.Failsafe);
legend0('MainState', 'NavState', 'ArmingState', 'Failsafe');
figtool(param, '');
subplot(2,1,2)
plot(t_LOAD, LOAD.CPU * 100);
figtool(param, 'CPU Load [%]');

% Battery Status
figure('Name', 'Battery Status', 'NumberTitle', 'off');
subplot(3,1,1)
plot(t_BATT, BATT.V, 'g', t_BATT, BATT.VFilt, 'b');
legend0('Raw', 'Filtered');
figtool(param, 'Voltage [V]');
subplot(3,1,2)
plot(t_BATT, BATT.C, 'g', t_BATT, BATT.CFilt, 'b');
legend0('Raw', 'Filtered');
figtool(param, 'Current [A]');
subplot(3,1,3)
plot(t_BATT, BATT.Remaining, t_BATT, BATT.Warning);
legend0('Remaining', 'Warning');
figtool(param, '');

% Onboard Power Status
figure('Name', 'Onboard Power Status', 'NumberTitle', 'off');
subplot(4,1,1)
plot(t_PWR, PWR.Periph5V);
figtool(param, 'Peripheral 5V [V]');
subplot(4,1,2)
plot(t_PWR, PWR.Servo5V);
figtool(param, 'Servo 5V [V]');
subplot(4,1,3)
plot(t_PWR, PWR.RSSI);
figtool(param, 'RSSI Volt. [V]');
subplot(4,1,4)
hold on
plot(t_PWR, PWR.UsbOk, '-b');
plot(t_PWR, PWR.BrickOk  * 0.98 + 0.01, '-r');
plot(t_PWR, PWR.ServoOk  * 0.96 + 0.02, '-g');
plot(t_PWR, PWR.PeriphOC * 0.94 + 0.03, '-c');
plot(t_PWR, PWR.HipwrOC  * 0.92 + 0.04, '-m');
legend0('USB OK', 'Brick OK', 'Servo OK', 'Peripheral OverCurrent', 'HiPower Periph. O.C.');
figtool(param, '');


% Telemetry Status
% figure('Name', 'Telemetry Status', 'NumberTitle', 'off');
% subplot(3,1,1)
% plot(t_TEL0, TEL0.RSSI, 'g', t_TEL0, TEL0.RemRSSI, 'b');

% figure('Name', 'RC Input', 'NumberTitle', 'off');
% subplot(4,1,3)
% plot(t_RC, RC.RSSI);
% subplot(4,1,4)
% plot(t_RC, RC.Drop);



end % end of function


function param = parse_arg(varargin)
    param.time_marked  = [];
    param.t_start      = [];
    param.t_start_from = 0;
    param.t_end        = [];
    param.t_end_from   = 0;

    for i = 1 : length(varargin)
        arg = varargin{i};
        if ischar(arg) && arg(1) == '|' && arg(end) == 's'
            % '|XXXs'
            
            param.time_marked(end+1) = str2double(arg(2:end-1));
        elseif ischar(arg) && ~isempty(strfind(arg, ':'))
            % 'XXXs:YYYs'
            % 'start:end'
            % 'start+AAAs:end-BBBs'
            
            n = strfind(arg, ':');
            
            left  = arg(1:n-1);
            right = arg(n+1:end);
            
            if strncmp(left, 'start+', 6) && left(end) == 's'
                param.t_start      = [];
                param.t_start_from = str2double(left(7:end-1));
            elseif strcmp(left, 'start') == 1
                param.t_start      = [];
                param.t_start_from = 0;
            elseif left(end) == 's'
                param.t_start      = str2double(left(1:end-1));
                param.t_start_from = [];
            end
            
            if strncmp(right, 'end-', 4) == 1 && right(end) == 's'
                param.t_end      = [];
                param.t_end_from = str2double(right(5:end-1));
            elseif strcmp(right, 'end') == 1
                param.t_end      = [];
                param.t_end_from = 0;
            elseif right(end) == 's'
                param.t_end      = str2double(right(1:end-1));
                param.t_end_from = [];
            end
        end
    end
end



function figtool(param, label_y)
    xlim(param.t_range);
    xlabel('Time [s]');
    ylabel(label_y);

    hold on
    lim = get(gca,'ylim');
    % avoid a bug (octave)
    if abs(lim(1)) < 1e-100 && abs(lim(2)) < 1e-100
        if lim(1) >= 0 && lim(2) >= 0
            lim = [0 1];
        elseif lim(1) < 0 && lim(2) < 0
            lim = [-1 0];
        else
            lim = [-1 1];
        end
    end
    ylim(lim);

    for i = 1 : length(param.time_marked)
        plot([param.time_marked(i) param.time_marked(i)], lim, ':k');
    end
end


function h = legend0(varargin)
    h = legend(varargin{:}, 'Location','northwest');
    set(h, 'FontSize', 8);
    set(h, 'color', 'none');
    legend boxoff
end


function i_Found = findindex(t_Src, t_Tgt)

    i_Found(1:length(t_Tgt)) = 0;

    for k = 1 : length(t_Tgt)
        if isempty(t_Src)
            i_Found(k) = 0;
            return;
        elseif length(t_Src) == 1
            i_Found(k) = 1;
            return;
        else
            % binary search
            i_left  = 1;
            i_right = length(t_Src);
            while 1
                i_mid = fix((i_left + i_right) / 2);
                if i_mid == i_left
                    % If the target value is placed between two values, rounds up
                    % index.
                    if t_Tgt(k) <= t_Src(i_left) 
                        i_Found(k) = i_left;
                    else
                        i_Found(k) = i_left + 1;
                    end
                    break;
                end
                if t_Tgt(k) < t_Src(i_mid)
                    i_right = i_mid;
                else
                    i_left = i_mid;
                end
            end
        end
    end
end


function n = getNRotors(log)
    MAV_TYPE = mavtypes();
    mav_type = getLogData(log, '.parm.MAV_TYPE');
    
    if ~isempty(mav_type)
        switch mav_type
            case MAV_TYPE.QUADROTOR
                n = 4;
            case MAV_TYPE.HEXAROTOR
                n = 6;
            case MAV_TYPE.OCTOROTOR
                n = 8;
            otherwise
                n = 8;
        end
    else
        n = 8;
    end
end


function MAV_TYPE = mavtypes()
    MAV_TYPE.GENERIC            = 0;
    MAV_TYPE.FIXED_WING         = 1;
    MAV_TYPE.QUADROTOR          = 2;
    MAV_TYPE.COAXIAL            = 3;
    MAV_TYPE.HELICOPTER         = 4;
    MAV_TYPE.ANTENNA_TRACKER    = 5;
    MAV_TYPE.GCS                = 6;
    MAV_TYPE.AIRSHIP            = 7;
    MAV_TYPE.FREE_BALLOON       = 8;
    MAV_TYPE.ROCKET             = 9;
    MAV_TYPE.GROUND_ROVER       = 10;
    MAV_TYPE.SURFACE_BOAT       = 11;
    MAV_TYPE.SUBMARINE          = 12;
    MAV_TYPE.HEXAROTOR          = 13;
    MAV_TYPE.OCTOROTOR          = 14;
    MAV_TYPE.TRICOPTER          = 15;
    MAV_TYPE.FLAPPING_WING      = 16;
    MAV_TYPE.KITE               = 17;
    MAV_TYPE.ONBOARD_CONTROLLER = 18;
    MAV_TYPE.VTOL_DUOROTOR      = 19;
    MAV_TYPE.VTOL_QUADROTOR     = 20;
    MAV_TYPE.VTOL_TILTROTOR     = 21;
    MAV_TYPE.VTOL_RESERVED2     = 22;
    MAV_TYPE.VTOL_RESERVED3     = 23;
    MAV_TYPE.VTOL_RESERVED4     = 24;
    MAV_TYPE.VTOL_RESERVED5     = 25;
    MAV_TYPE.GIMBAL             = 26;
    MAV_TYPE.ADSB               = 27;
end