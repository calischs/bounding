function animate_simple(t,R,C,speed,varargin)
% ANIMATE_SIMPLE animates simulation results
% t is a row vector of time points
% R is a matrix consisting of alternating rows of x and y coordinates of
%   animation points at each time point
% C is a vector in which each row contains the indices of two animation
%   points to be connected by a line
% speed is the speed at which the animation will play relative to real time

axis equal % sets the X:Y aspect ratio 1:1; otherwise things will look stretched

% sets the axes to fit everything at all times
axis([min(min(R(1:2:end,:))) max(max(R(1:2:end,:))) min(min(R(2:2:end,:))) max(max(R(2:2:end,:)))]);

% initialize the lines with the first column of R
legline = drawlines(R(:,1),C);

if isempty(varargin)
tic                                             % start counter
while toc < t(end)/speed                        % while there's real time left
    tsim = toc*speed;                           % determine the simulation time to draw
    Rint = interp1(t',R',tsim', 'linear')';     % interpolate to get coordinates at that time
    drawlines(Rint,C,legline);                  % draw the lines at that time
end
drawlines(R(:,end),C,legline);                  % be sure to end on the last frame

elseif strcmp(varargin{1},'save')
    fps = 30;                                         % playback speed
    tanim = linspace(t(1),t(end),round(t(end)*fps));  % set desired time points
    Ranim = interp1(t',R',tanim', 'linear')';         % interpolate point coordinates 
    
    for ii = 1:length(tanim)
        drawlines(Ranim(:,ii),C,legline);             % draw the lines at that time
        A(ii) = getframe(gca);                        % save the frame to an array of frames
    end
    movie2avi(A, 'test', 'Compression', 'Cinepak')    % save the array of frames to a movie
end
end

function legline = drawlines(Rvec,C,varargin)
% DRAWLINES draws a single frame of the animation
% Rvec is a column vector consisting of alternating rows of x and y
%   coordinates of animation points at a single time point.  That is, it is a
%   column selected from the R matrix.
% C contains the indices of the animation points to be connected
% varargin, when specified, contains the handles to the existing line
%   objects

% Format the X and Y data as required by line() function
X = Rvec(2*C-1);
Y = Rvec(2*C);

if isempty(varargin)
    % this is how to initialize line objects with multiple segments
    legline = line(X',Y','linewidth', 2,'color','blue', 'visible', 'on', 'marker','.','markersize',10, 'markeredgecolor','red');
else
    % this is how to update line objects with multiple segments
    legline = varargin{1};
    set(legline,{'xdata'},num2cell(X,2),{'ydata'}, num2cell(Y,2));
end

drawnow % if you don't call this function, it will only draw the last frame
end
