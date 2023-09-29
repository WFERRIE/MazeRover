function cellType = getLocalizationCell(u)
    numBlocked = 0;
    for k = 1:4
        if (u(k) < 15)
            numBlocked = numBlocked + 1;
        end        
    end
    
    if (numBlocked == 2)
        if ((u(1) > 4 && u(3) > 4) || (u(2) > 4 && u(4) > 4))
            cellType = 5;
        else 
            cellType = 2;
        end
    else
        cellType = numBlocked;
    end
end
