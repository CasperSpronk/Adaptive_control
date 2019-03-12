function amt = constrain_value (amt, low, high)
%%
    if (isnan(amt))
        amt = (low + high) / 2;
    end

    if (amt < low)
        amt = low;
    end

    if (amt > high)
        amt = high;
    end

end