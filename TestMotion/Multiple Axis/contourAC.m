function [] = contourAC(g,ydiff)
%CONTOUR Summary of this function goes here

%   Detailed explanation goes here

g.GCommand('AB') % Aborts programs
g.GCommand('SH AC') % servo motor A
g.GCommand('PA 0') % Set current position to 0
g.GMotionComplete('AC')% Wait for motion to complete
g.GCommand('WT100') % Wait before executing next commands

TargetBuff=100;
N=length(ydiff);
cmdArrays = ceil(N/TargetBuff)

g.GCommand('CMAC')
g.GCommand('CD 0,,0')
g.GCommand('DT 2')

posStr = "CD "+string(ydiff(:,1))+","+","+string(ydiff(:,2))+";";

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

        %java.lang.Thread.sleep(400) 
        %buffsize=g.GCommand('CM?')
        "Buffer_size"
        str2num(buffsize.string)
    end
    "Limit switch check"
    %if(switches(g))
    %    break
    %end

end

g.GCommand('CD 0,,0=0') % end of counter buffer
%g.GCommand('#Wait;JP#Wait,_CM<>511')% Wait until path is done

end

