function [] = StreamContourData_to_Galil_w_trigger(g,hex_path)

yy=hex_path.axis_cts';
ydiff=diff(round(yy)); %Relative move commands sent to contour buffer

DT_g=round(log2(hex_path.dt*1024));
%CONTOUR
g.GInfo;
g.GCommand('ST');
g.GCommand('SH ABCEFG'); % servo motors ABCEFG

TargetBuff=250;
N=length(ydiff);
cmdArrays = ceil(N/TargetBuff)

g.GCommand('CMABCEFG');

g.GCommand(['DT ' num2str(DT_g)])

posStr = "CD "+string(ydiff(:,1))+","+string(ydiff(:,2))+","+...
    string(ydiff(:,3))+","+","+string(ydiff(:,4))+","+string(ydiff(:,5))+...
    ","+string(ydiff(:,6))+";";

n=1;
i=1;
j=0;

% Buffer-health monitoring (see StreamContourData_to_Galil.m for notes).
BUFFER_CAPACITY      = 511;
STARVATION_THRESHOLD = 50;
buf_min_pending      = Inf;

while n<cmdArrays+1
    buffsize=g.GCommand('CM?');
    free_val    = str2num(buffsize.string); %#ok<ST2NM>
    pending_val = BUFFER_CAPACITY - free_val;
    if pending_val < buf_min_pending
        buf_min_pending = pending_val;
    end
    if free_val >= TargetBuff

        if(length(posStr)<j+TargetBuff)

            command =strjoin(posStr(i:end,1));
        else
            command =strjoin(posStr((i):(j+TargetBuff),1));
        end
        g.GCommand(command); % CD specifies the incremental position

        n=n+1;
        i=i+TargetBuff;
        j=j+TargetBuff;
    end
end


buffsizen=1;
while buffsizen~=511
    drawnow   % yield so timer callbacks (encoder DROs etc.) fire during drain
    buffsize=g.GCommand('CM?');
    buffsizen=str2num(buffsize.string);
end
g.GCommand('CD 0,0,0,,0,0,0=0'); % end of counter buffer
g.GCommand('ST');

if isfinite(buf_min_pending) && buf_min_pending < STARVATION_THRESHOLD
    warning('StreamContourData_to_Galil_w_trigger:BufferLow', ...
        ['Contour buffer pending dropped to %d samples during stream ', ...
         '(starvation threshold %d). On real hardware this is close ', ...
         'to a buffer underrun.'], ...
        buf_min_pending, STARVATION_THRESHOLD);
end

end

