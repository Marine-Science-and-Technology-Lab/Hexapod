function [] = contourABCEFG(g,ydiff)
%CONTOUR 

% g.GCommand('AB') % Aborts programs
 g.GCommand('ST')
g.GCommand('SH ABCEFG') % servo motor A
% g.GCommand('PA 0') % Set current position to 0
% g.GMotionComplete('ABCEFG')% Wait for motion to complete
g.GCommand('WT250') % Wait before executing next commands
pause(0.5)

TargetBuff=205;
N=length(ydiff);
cmdArrays = ceil(N/TargetBuff)


g.GCommand('CMABCEFG')
% g.GCommand('CD 0,0,0,,0,0,0')
g.GCommand('DT 2')

posStr = "CD "+string(ydiff(:,1))+","+string(ydiff(:,2))+","+...
    string(ydiff(:,3))+","+","+string(ydiff(:,4))+","+string(ydiff(:,5))+...
    ","+string(ydiff(:,6))+";";

n=1;
i=1;
j=0;

while n<cmdArrays+1
    buffsize=g.GCommand('CM?');

    if(str2num(buffsize.string) >= TargetBuff)

        if(length(posStr)<j+TargetBuff)

            command =strjoin(posStr(i:end,1));
        else
            command =strjoin(posStr((i):(j+TargetBuff),1));
        end
        g.GCommand(command); % CD specifies the incremental position

        n=n+1;
        i=i+TargetBuff;
        j=j+TargetBuff;
    else
        % Print buffer size
        %"Buffer_size"
        %str2num(buffsize.string)
    end

end


buffsizen=1;
while buffsizen~=511
    buffsize=g.GCommand('CM?');
    buffsizen=str2num(buffsize.string);
end
g.GCommand('CD 0,0,0,,0,0,0=0') % end of counter buffer
g.GCommand('ST')
%  g.GMotionComplete('ABCEFG')
g.GCommand('PA 0,0,0,,0,0,0')
g.GCommand('BGABCEFG')

end

