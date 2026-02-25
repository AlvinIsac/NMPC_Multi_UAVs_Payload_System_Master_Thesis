function tileFigs(rows, cols, varargin)
% tileFigs(rows, cols, 'Padding',8, 'Monitor',1, 'Margins',[10 40 10 50])
% Places all open MATLAB figure windows in a rows×cols grid with **no overlap**.
% Uses the figure **OuterPosition** (includes title bar + borders).

% ---- options ----
p = inputParser;
addParameter(p,'Padding',6);                 % space between cells (px)
addParameter(p,'Monitor',1);                 % 1 = primary
addParameter(p,'Margins',[8 35 8 45]);       % [left bottom right top] px
parse(p,varargin{:});
pad = p.Results.Padding;
mIdx = p.Results.Monitor;
marg = p.Results.Margins;

% ---- figures (sorted by figure number) ----
figs = findall(0,'Type','figure','Visible','on');
if isempty(figs), return; end
[~,ix] = sort([figs.Number]);
figs = figs(ix);

% ---- monitor bounds ----
mp = get(0,'MonitorPositions');  % [x y w h] per monitor
mIdx = min(mIdx, size(mp,1));
mon = mp(mIdx,:);

% usable area (account for menu bar/dock with margins)
usableW = mon(3) - (marg(1) + marg(3));
usableH = mon(4) - (marg(2) + marg(4));

% cell size (exact integers; subtract inter-cell padding)
cellW = floor((usableW - pad*(cols-1)) / cols);
cellH = floor((usableH - pad*(rows-1)) / rows);

% ---- place each figure ----
n = numel(figs);
for k = 1:n
    r = floor((k-1)/cols);         % 0-based row, top→bottom
    c = mod(k-1, cols);            % 0-based col, left→right

    % y from bottom; flip rows to start at the top
    x = mon(1) + marg(1) + c*(cellW + pad);
    y = mon(2) + marg(2) + (rows-1-r)*(cellH + pad);

    set(figs(k), 'Units','pixels', 'WindowStyle','normal');
    % Use OuterPosition to include window chrome → prevents overlap
    set(figs(k), 'OuterPosition', [x, y, cellW, cellH]);
end
end
