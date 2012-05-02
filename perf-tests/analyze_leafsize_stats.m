function [] = analyze_leafsize_stats()
	% Compute the stats from the result files of performance tests wrt 
	% the max. leaf size
    close all;
    
    %D=load('LEAF_STATS.txt');
    %D=load('LEAF_STATS_DOUBLE.txt');
    D=load('LEAF_STATS_DATASET.txt');
	MAXs = unique(D(:,2));
    
    COLs = {'k','b','r','g'};  % Cell array of colros.
    
    figure(1);
    for i=1:length(MAXs),
        MaxLeaf = MAXs(i);
		idxs = find(D(:,2)==MaxLeaf);
        
        TBs = D(idxs,3); % Time of tree Builds
        TQs = D(idxs,4); % Time of tree Queries
        
        TBm   = 1e3*mean(TBs);
        TQm   = 1e6*mean(TQs);
        TBstd = 1e3*std(TBs);
        TQstd = 1e6*std(TQs);
                
        TBms(i)=TBm;
        TQms(i)=TQm;
        
        % Plot a 95% ellipse:
        my_plot_ellipse([TBm TQm],2*[TBstd TQstd], COLs{ 1+mod(i-1,length(COLs))} );
        
        %plot(1e3*TBs,1e6*TQs,'.','color',COLs{ 1+mod(i-1,length(COLs))});
        
        text(TBm,TQm+6*TQstd,sprintf('%i',MaxLeaf));
    end
    
    plot(TBms,TQms,'color',[0.4 0.4 0.4],'LineWidth',2.5);
        
    set(gca,'YScale','log');
    
    ylabel('Tree query time (us)');
    xlabel('Tree build time (ms)');
    title(sprintf('nanoflann performance vs. Leaf Max.Size (#points=%.01e)',D(1,1)));
    
end


function [] = my_plot_ellipse(Ms,STDs, COL )
    angs=linspace(0,2*pi,200);
    xs= Ms(1) + cos(angs)*STDs(1);
    ys= Ms(2) + sin(angs)*STDs(2);
    h=plot(xs,ys,'color', COL,'LineWidth',2);
    hold on;
end