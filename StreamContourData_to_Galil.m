function [] = StreamContourData_to_Galil(g,hex_path)

yy=hex_path.axis_cts';
ydiff=diff(round(yy)); %Relative move commands sent to contour buffer

DT_g=round(log2(hex_path.dt*1024));
%CONTOUR 
g.GInfo
g.GCommand('CO 15')
 g.GCommand('ST')
g.GCommand('SH ABCEFG') % servo motors ABCEFG

TargetBuff=250;
N=length(ydiff);
cmdArrays = ceil(N/TargetBuff)


posStr = "CD "+string(ydiff(:,1))+","+string(ydiff(:,2))+","+...
    string(ydiff(:,3))+","+","+string(ydiff(:,4))+","+string(ydiff(:,5))+...
    ","+string(ydiff(:,6))+";";

CMD2=sprintf('#Pulse; \n #A; \n SB 25; \n SB 17; \n WT64,1; \n CB 25; \n CB 17; \n WT64,1; \n JP #A; \n CB 25; \n CB 17; \n EN');

g.GProgramDownload(CMD2);
g.GCommand('XQ #Pulse,2');
g.GCommand('CMABCEFG')

g.GCommand(['DT ' num2str(DT_g)])

n=1;
i=1;
j=0;

% hwait=waitbar(0,'Streaming Coordinates To Galil')


while n<cmdArrays+1
    buffsize=g.GCommand('CM?');
% waitbar(n/(cmdArrays+1))
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
% if ~ishandle(hwait)
%     break
% end
end


buffsizen=1;
while buffsizen~=511
    buffsize=g.GCommand('CM?');
    buffsizen=str2num(buffsize.string);
%     waitbar(buffsizen/(511),'Draining Buffer');
%     if ~ishandle(hwait)
%     break
% end
end
g.GCommand('CD 0,0,0,,0,0,0=0') % end of counter buffer
g.GCommand('CB25')
g.GCommand('CB17')
g.GCommand('ST')

%  g.GMotionComplete('ABCEFG')
% g.GCommand('PA 0,0,0,,0,0,0')
% g.GCommand('BGABCEFG')

end

