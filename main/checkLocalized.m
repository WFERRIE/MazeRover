function localized = checkLocalized(p, threshholdUpper, threshholdLower, count)
    if (sum(p > threshholdUpper, 'all') >= count) && (sum(p > threshholdLower, 'all') >= count),
%     if (3 <= sum(0.04 < p < 0.07, 'all') <= )
        disp('localized!')
        localized = 1;
    else,
        localized = 0;
    end
  
end