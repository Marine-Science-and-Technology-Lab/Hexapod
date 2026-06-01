function fig = AnimateHexapod(hex_obj, hex_setup, hex_path)
% AnimateHexapod - Popup 3D playback of a programmed hexapod path.
%
%   fig = AnimateHexapod(hex_obj, hex_setup, hex_path)
%
%   Opens a new figure showing the hexapod rendered in 3D, with a
%   scrubbable time slider and Play / Pause / Stop / speed / loop
%   transport controls. Uses hexapodGraphic_nocalc_replot.m to update
%   the 3D primitives in place on each frame.
%
%   Inputs
%     hex_obj    - hexapod object (from InitializeHexapodObject)
%     hex_setup  - setup struct (used only indirectly through IK)
%     hex_path   - path struct with .pose_t (6xN), .dt, .T
%
%   Outputs
%     fig        - handle to the created figure. Its timer is stopped
%                  and deleted automatically when the figure is closed.
%
%   Called from the HexControl "Animate Motion" button.

if ~isfield(hex_path, 'pose_t') || isempty(hex_path.pose_t)
    errordlg('No motion profile loaded. Generate or import a path first.', ...
        'Animate Motion');
    fig = [];
    return
end

N_frames = size(hex_path.pose_t, 2);
if N_frames < 2
    errordlg('Motion profile must have at least 2 samples.', 'Animate Motion');
    fig = [];
    return
end

if ~isfield(hex_path, 'dt') || isempty(hex_path.dt)
    hex_path.dt = 2^2 / 1024;   % default DT=2 at 1024 Hz base
end

% --- Build figure + controls ----------------------------------------
fig = figure( ...
    'Name', 'Animate Motion', ...
    'NumberTitle', 'off', ...
    'Position', [100 100 900 720], ...
    'Color', [0.95 0.95 0.95], ...
    'CloseRequestFcn', @onClose);

ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0.05 0.18 0.9 0.78]);
title(ax, 'Hexapod motion playback');
xlabel(ax, 'X'); ylabel(ax, 'Y'); zlabel(ax, 'Z');
view(ax, 40, 25);

panelH = 0.13;
panel = uipanel('Parent', fig, 'Units', 'normalized', ...
    'Position', [0.02 0.01 0.96 panelH], 'Title', 'Playback');

% Transport buttons
btnRewind = uicontrol(panel, 'Style', 'pushbutton', 'String', '|<<', ...
    'Units', 'normalized', 'Position', [0.01 0.2 0.06 0.6], ...
    'TooltipString', 'Jump to start');
btnPlay = uicontrol(panel, 'Style', 'togglebutton', 'String', 'Play', ...
    'Units', 'normalized', 'Position', [0.08 0.2 0.07 0.6], ...
    'FontWeight', 'bold');
btnEnd = uicontrol(panel, 'Style', 'pushbutton', 'String', '>>|', ...
    'Units', 'normalized', 'Position', [0.16 0.2 0.06 0.6], ...
    'TooltipString', 'Jump to end');

% Time slider
slider = uicontrol(panel, 'Style', 'slider', ...
    'Min', 1, 'Max', N_frames, 'Value', 1, ...
    'SliderStep', [1/(N_frames-1) max(10, N_frames/50)/(N_frames-1)], ...
    'Units', 'normalized', 'Position', [0.23 0.3 0.45 0.4]);

% Time readout
timeLabel = uicontrol(panel, 'Style', 'text', 'String', '', ...
    'Units', 'normalized', 'Position', [0.69 0.25 0.1 0.5], ...
    'FontName', 'Consolas', 'HorizontalAlignment', 'center', ...
    'BackgroundColor', [0.95 0.95 0.95]);

% Speed selector
uicontrol(panel, 'Style', 'text', 'String', 'Speed:', ...
    'Units', 'normalized', 'Position', [0.8 0.55 0.05 0.3], ...
    'HorizontalAlignment', 'right', 'BackgroundColor', [0.95 0.95 0.95]);
speedChoices = {'0.1x', '0.25x', '0.5x', '1x', '2x', '4x', '10x'};
speedValues  = [0.1 0.25 0.5 1 2 4 10];
speedPopup = uicontrol(panel, 'Style', 'popupmenu', 'String', speedChoices, ...
    'Value', 4, ... % default 1x
    'Units', 'normalized', 'Position', [0.85 0.55 0.08 0.3]);

% Loop toggle
loopCb = uicontrol(panel, 'Style', 'checkbox', 'String', 'Loop', ...
    'Value', 0, ...
    'Units', 'normalized', 'Position', [0.85 0.15 0.08 0.3], ...
    'BackgroundColor', [0.95 0.95 0.95]);

% --- Initialize 3D render using hexapodGraphic_nocalc_replot --------
% Local copy so we don't stomp on the caller's hex_obj.
hex_local = hex_obj;
plot3D = struct('ax', ax);

% Initial frame (index 1)
[hex_local, plot3D] = renderFrame(hex_local, hex_setup, hex_path, plot3D, 1);
axis(ax, 'equal');
grid(ax, 'on');
view(ax, 40, 25);

% --- Timer ----------------------------------------------------------
targetFPS = 30;
t = timer( ...
    'ExecutionMode', 'fixedSpacing', ...
    'Period', 1/targetFPS, ...
    'BusyMode', 'drop');
t.TimerFcn = @(tObj, evt) onTimer(tObj, evt, fig);

% --- Stash state on the figure so callbacks can reach it -----------
state = struct();
state.fig         = fig;
state.ax          = ax;
state.hex_obj     = hex_local;
state.hex_setup   = hex_setup;
state.hex_path    = hex_path;
state.plot3D      = plot3D;
state.N_frames    = N_frames;
state.currentFrame = 1;
state.timer        = t;
state.targetFPS    = targetFPS;
state.speedPopup   = speedPopup;
state.speedValues  = speedValues;
state.loopCb       = loopCb;
state.slider       = slider;
state.btnPlay      = btnPlay;
state.timeLabel    = timeLabel;
state.playing      = false;
fig.UserData = state;

updateTimeLabel(fig);

% --- Wire callbacks now that state exists --------------------------
btnRewind.Callback = @(src,evt) jumpTo(fig, 1);
btnEnd.Callback    = @(src,evt) jumpTo(fig, state.N_frames);
btnPlay.Callback   = @(src,evt) togglePlay(fig);
slider.Callback    = @(src,evt) onScrub(fig, round(src.Value));

end


% =====================================================================
%                              Callbacks
% =====================================================================

function onTimer(~, ~, fig)
    if ~isvalid(fig); return; end
    s = fig.UserData;
    if ~s.playing; return; end

    speed = s.speedValues(s.speedPopup.Value);
    framesPerTick = max(1, round(speed * (1/s.targetFPS) / s.hex_path.dt));

    newFrame = s.currentFrame + framesPerTick;
    if newFrame > s.N_frames
        if s.loopCb.Value
            newFrame = mod(newFrame - 1, s.N_frames) + 1;
        else
            newFrame = s.N_frames;
            s.playing = false;
            s.btnPlay.Value = 0;
            s.btnPlay.String = 'Play';
        end
    end
    s.currentFrame = newFrame;
    fig.UserData = s;

    applyFrame(fig, newFrame);
end


function togglePlay(fig)
    s = fig.UserData;
    if s.btnPlay.Value == 1
        s.playing = true;
        s.btnPlay.String = 'Pause';
        % If we're at the end and not looping, rewind to start on play.
        if s.currentFrame >= s.N_frames && ~s.loopCb.Value
            s.currentFrame = 1;
        end
        fig.UserData = s;
        if strcmp(s.timer.Running, 'off')
            start(s.timer);
        end
    else
        s.playing = false;
        s.btnPlay.String = 'Play';
        fig.UserData = s;
    end
end


function jumpTo(fig, frame)
    s = fig.UserData;
    s.playing = false;
    s.btnPlay.Value = 0;
    s.btnPlay.String = 'Play';
    s.currentFrame = max(1, min(frame, s.N_frames));
    fig.UserData = s;
    applyFrame(fig, s.currentFrame);
end


function onScrub(fig, frame)
    s = fig.UserData;
    % Pause on scrub; user can Play again to resume from scrub point.
    s.playing = false;
    s.btnPlay.Value = 0;
    s.btnPlay.String = 'Play';
    s.currentFrame = max(1, min(frame, s.N_frames));
    fig.UserData = s;
    applyFrame(fig, s.currentFrame);
end


function applyFrame(fig, frame)
    if ~isvalid(fig); return; end
    s = fig.UserData;
    [s.hex_obj, s.plot3D] = renderFrame(s.hex_obj, s.hex_setup, s.hex_path, ...
        s.plot3D, frame);
    s.slider.Value = frame;
    fig.UserData = s;
    updateTimeLabel(fig);
    drawnow limitrate
end


function updateTimeLabel(fig)
    s = fig.UserData;
    t = (s.currentFrame - 1) * s.hex_path.dt;
    t_total = (s.N_frames - 1) * s.hex_path.dt;
    s.timeLabel.String = sprintf('%5.2f / %5.2f s\n%d / %d', ...
        t, t_total, s.currentFrame, s.N_frames);
end


function onClose(src, ~)
    try
        s = src.UserData;
        if isfield(s, 'timer') && isvalid(s.timer)
            stop(s.timer);
            delete(s.timer);
        end
    catch
        % If the UserData is malformed, nothing to clean up.
    end
    delete(src);
end


% =====================================================================
%                          Per-frame renderer
% =====================================================================

function [hex_obj, plot3D] = renderFrame(hex_obj, hex_setup, hex_path, plot3D, frame)
    hex_obj.pose = hex_path.pose_t(:, frame);
    hex_obj = InverseKinematics_hexapod(hex_obj, hex_setup);
    plot3D = hexapodGraphic_nocalc_replot(hex_obj, plot3D);
end
