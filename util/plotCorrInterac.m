function plotCorrInterac(corr)
    % Create a colour plot of the input corr matrix, using a slider to
    % control which timestep to show
    % Input args:
    %   corr - a (n,n,num_iter) thing where the 3rd dim is the timestep

    NUM_ITER = size(corr, 3);
    
    init_k = 1;
    S.fh = figure('units','pixels',...
                  'name','slider_plot',...
                  'numbertitle','off',...
                  'position',[20 80 630 630], ...
                  'resize','on');
    S.k = 1:NUM_ITER;  % For plotting. TODO I think I understood this wrong     
    S.ax = axes('unit','pix',...
                'position',[20 80 560 510]); % TODO update these things?
    S.im = imagesc(corr(:,:,init_k), [-1, 1]);
%     S.cb = colorbar('position',[600 80 30 510]);
    S.cb = colorbar();
    S.title = title(get_title_str(init_k));
    S.sl = uicontrol('style','slide',...
                     'unit','pix',...
                     'position',[20 10 560 30],...
                     'min',1,'max',NUM_ITER,'val',init_k,...
                     'sliderstep',[1/(NUM_ITER-1) 1/(NUM_ITER-1)],...
                     'callback',{@sl_call,S});
                 
    function [] = sl_call(varargin)
        % Callback for the slider.
        % TODO change imagesc's CData field
        [h,S] = varargin{[1,3]};  % calling handle and data structure.
        k = round(get(h,'value'));
        S.im.CData = corr(:,:,k); % update data displayed
        S.title.String = get_title_str(k);
%         set(S.LN,'ydata',S.k.^get(h,'value'))
    end

    function title_str = get_title_str(k)
        title_str = sprintf('Correlation matrix at iteration %d', k);
    end
end

