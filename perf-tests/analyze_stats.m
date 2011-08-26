function [] = analyze_stats()
	% Compute the stats from the result files of flann & nanoflann performance tests
	%
    close all;
    
	[Nsf, Tf_M, Tf_STD]   = analyze_file('stats_flann.txt');
	[Nsnf, Tnf_M, Tnf_STD]= analyze_file('stats_nanoflann.txt');

    
    titles={'Convert into Matrix<>', 'Build index', 'One 3D query'};

    T_factors=[1e3, 1, 1e6];
    t_titles={'Time (ms)','Time (s)','Time (\mu s)'};

    for figs=1:3,
        figure(figs);
        % This is just to fix the legend:
        plot(0,0,'r'); hold on; plot(0,0,'b');
        
        f = T_factors(figs);
        
        plotMeansAndStd( Nsf, f*Tf_M(:,figs), f*Tf_STD(:,figs), 'r', 2.0, 1, 10 );
        plotMeansAndStd( Nsnf, f*Tnf_M(:,figs), f*Tnf_STD(:,figs), 'b', 2.0, 1, 10 );
        set(gca,'XScale','Log')
        title(titles{figs});
        xlabel('Size of point cloud');
        ylabel(t_titles{figs});
        legend('flann','nanoflann', 'Location', 'NorthWest');
    end
    
    % Fig: time saved in building the index:
    figure(4);
    figs = 2;
    plotMeansAndStd( Nsf, 1e3*(Tf_M(:,1) + Tf_M(:,figs)-Tnf_M(:,figs)), 1e3*sqrt((Tf_STD(:,figs)+Tnf_STD(:,figs)).^2), 'k', 2.0, 1, 10 );
    set(gca,'XScale','Log')
    title('Time saved in index building with nanoflann vs. flann (incl. the no matrix time)');
    xlabel('Size of point cloud');
    ylabel('Time (ms)');
    
end


function [Ns, T_M, T_STD] = analyze_file(fil)
	D=load(fil);
	Ns = unique(D(:,1));
	nCols = size(D,2)-1;
	for i=1:length(Ns),
		idxs = find(D(:,1)==Ns(i));
		for j=1:nCols,
			T_M(i,j)=mean(D(idxs,j+1));
			T_STD(i,j)=std(D(idxs,j+1));
		end
	end

end

function [] = plotMeansAndStd( x, y_m, y_std, style, stdMult,doDrawMeanLine,smallSegmentSize)
	% -------------------------------------------------------------
	%    function [] = plotMeansAndStd( x, y_m, y_std, style, stdMult,doDrawMeanLine,smallSegmentSize)
	%
	%  Plots a graph (x,y_m), plus the uncertainties "y_std" segments 
	%
	%  Jose Luis Blanco Claraco, 28-JUN-2006
	% -------------------------------------------------------------
	if (doDrawMeanLine),
	    plot(x,y_m,style); 
	end
	hold on;

	for i=1:length(x),
	    y1 = y_m(i) - stdMult*y_std(i);
	    y2 = y_m(i) + stdMult*y_std(i);
	    plot([x(i)-smallSegmentSize x(i)+smallSegmentSize x(i) x(i) x(i)-smallSegmentSize x(i)+smallSegmentSize],[y1 y1 y1 y2 y2 y2],style);
	end
end
