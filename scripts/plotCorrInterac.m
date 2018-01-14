function plotCorrInterac(corr, folder_name_to_save)
    % Create a colour plot of the input corr matrix, using a slider to
    % control which timestep to show. Optionally saves figure to folder
    % Input args:
    %   corr - a (n,n,num_iter) thing where the 3rd dim is the timestep
    %   folder_name_to_save - folder name. Provide this if you want to save
    %                         the folder

    NUM_ITER = size(corr, 3);
    
    init_k = 1;
    S.fh = figure('units','pixels',...
                  'name','slider_plot',...
                  'numbertitle','off',...
                  'position',[20 80 630 630], ...
                  'resize','on');
    S.ax = axes('unit','pix',...
                'position',[20 80 560 510]);
    S.im = imagesc(corr(:,:,init_k), [-1, 1]);
    S.cb = colorbar();
    S.title = title(get_title_str(init_k));
    S.sl = uicontrol('style','slide',...
                     'unit','pix',...
                     'position',[20 10 560 30],...
                     'min',1,'max',NUM_ITER,'val',init_k,...
                     'sliderstep',[1/(NUM_ITER-1) 1/(NUM_ITER-1)],...
                     'callback',{@sl_call,S});
    
    if exist('folder_name_to_save', 'var')
        saveas(gcf, strcat(folder_name_to_save, 'corr_matrix.fig'));
    end
                 
    function [] = sl_call(varargin)
        % Callback for the slider.
        [h,S] = varargin{[1,3]};  % calling handle and data structure.
        k = round(get(h,'value'));
        S.im.CData = corr(:,:,k); % update data displayed
        S.title.String = get_title_str(k);
    end

    function title_str = get_title_str(k)
        title_str = sprintf('Correlation matrix at iteration %d', k);
    end
end

