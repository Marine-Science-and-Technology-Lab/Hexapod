function [MSH_A, MSH_B, MSH_C, MSH_D] = importMeshes(meshDir)
% importMeshes - Load yoke collision meshes from STL files.
%
%   [MSH_A, MSH_B, MSH_C, MSH_D] = importMeshes()
%       Auto-discovers STLs (YokeA_*.STL, YokeB_*.STL, YokeC_*.STL,
%       YokeD_*.STL) in this script's own directory.
%
%   [...] = importMeshes(meshDir)
%       Searches meshDir for the four yoke groups instead.
%
% If a group has no matching STLs in the chosen directory, a single
% uigetfile multiselect dialog is opened as a fallback for that group.
% When called with no output arguments, the four groups are rendered
% for visual inspection.

if nargin < 1 || isempty(meshDir)
    meshDir = fileparts(mfilename('fullpath'));
end

MSH_A = loadYokeGroup(meshDir, 'YokeA_*.STL', 'YokeA');
MSH_B = loadYokeGroup(meshDir, 'YokeB_*.STL', 'YokeB');
MSH_C = loadYokeGroup(meshDir, 'YokeC_*.STL', 'YokeC');
MSH_D = loadYokeGroup(meshDir, 'YokeD_*.STL', 'YokeD');

if nargout == 0
    figure; hold on; axis equal;
    plotGroup(MSH_A, 'b');
    plotGroup(MSH_B, 'r');
    figure; hold on; axis equal;
    plotGroup(MSH_C, 'b');
    plotGroup(MSH_D, 'r');
end
end


function MSH = loadYokeGroup(meshDir, pattern, label)
files = dir(fullfile(meshDir, pattern));
if isempty(files)
    [fnames, fpath] = uigetfile(fullfile(meshDir, pattern), ...
        sprintf('Select STLs for %s (no matches for %s)', label, pattern), ...
        'MultiSelect', 'on');
    if isequal(fnames, 0)
        MSH = [];
        return
    end
    if ischar(fnames)
        fnames = {fnames};
    end
    files = repmat(struct('name', '', 'folder', fpath), 1, length(fnames));
    for n = 1:length(fnames)
        files(n).name = fnames{n};
        files(n).folder = fpath;
    end
end

for n = 1:length(files)
    yoke = stlread_fex(fullfile(files(n).folder, files(n).name));
    MSH(n) = collisionMesh(yoke.vertices); %#ok<AGROW>
end
end


function plotGroup(MSH, faceColor)
for n = 1:length(MSH)
    [~, pt] = show(MSH(n));
    pt.FaceColor = faceColor;
end
end
