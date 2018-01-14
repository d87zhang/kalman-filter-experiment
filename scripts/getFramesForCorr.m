% Creates struct array of frames showing the imagesc plots of the corr
% matrix. Stores said struct array in variable corr_frames. This struct
% array can be directly consumed by movie()
assert(NUM_ITER == size(corr, 3));

% select timesteps to include
FRAME_SKIP = 5;
time_steps = 1:FRAME_SKIP:NUM_ITER;

tic
% somehow you can't close with the returned handle... so give it a name
fig = figure('Name', 'secretFrame', 'visible','on');
ax = gca;

clear corr_frames;
corr_frames(length(time_steps)) = struct('cdata',[],'colormap',[]);
first = false;
for i = 1:length(time_steps)
    k = time_steps(i);
    imagesc(corr(:,:,k), [-1, 1]);
    if first
%         ax.NextPlot = 'replaceChildren';
        first = false;
    end
    colorbar;
    title(sprintf('Correlation matrix at iteration %d', k));
    drawnow
    
    corr_frames(i) = getframe(fig);
end
close secretFrame;
toc

% Play the movie
% figure('units','normalized','outerposition',[0 0 1 1]);
% movie(corr_frames, 1, 15);

%% write the video to file
disp('Note: uncomment code below this disp call() to save the video');
% vid_writer = VideoWriter([tempFolderName, 'corr over time.avi']);
% open(vid_writer);
% writeVideo(vid_writer, corr_frames);
% close(vid_writer);

