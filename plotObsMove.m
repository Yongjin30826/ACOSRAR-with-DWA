function plotObsMove(xy,r)
% ª≠’œ∞≠ŒÔ£¨
 th = 0:0.05:2*pi;
 xunit = r * cos(th) + xy(1);
 yunit = r * sin(th) + xy(2);
 d = plot(xunit, yunit);
end