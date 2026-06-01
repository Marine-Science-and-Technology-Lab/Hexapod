function g = galilFactory(emulated, address)
% galilFactory - Return a real Galil controller or an emulator.
%
%   g = galilFactory(emulated)
%   g = galilFactory(emulated, address)
%
%   If emulated is true, returns a GalilEmulator and opens it. Otherwise
%   returns a real py.gclib.py() and opens the given address. In both
%   cases the returned handle exposes the same gclib surface used
%   throughout the codebase (GCommand, GProgramDownload, GInfo,
%   GMotionComplete, GOpen, GClose), so callers don't need a mode switch.
%
%   Use from HexControl's InitializeControllerButtonPushed callback:
%       app.g = galilFactory(app.Emulated, app.ControllerIP);

if nargin < 2
    address = '';
end

if emulated
    g = GalilEmulator();
    if isempty(address)
        address = 'emulator';
    end
    g.GOpen(address);
else
    g = py.gclib.py();
    g.GOpen(address);
end
end
