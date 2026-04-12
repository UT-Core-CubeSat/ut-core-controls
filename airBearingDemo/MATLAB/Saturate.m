function input = Saturate(input, lower_limit, upper_limit)

    if isempty(upper_limit)
        upper_limit =  abs(lower_limit);
        lower_limit = -abs(lower_limit);
    end

    if isscalar(lower_limit)
        lower_limit = repmat(lower_limit, size(input));
    else
        assert(all(size(input)==size(lower_limit)), "Input and limits must be the same size.")
    end
    if isscalar(upper_limit)
        upper_limit = repmat(upper_limit, size(input));
    else
        assert(all(size(input)==size(upper_limit)), "Input and limits must be the same size.")
    end

    % For each entry
    for i = 1:length(input)
        % If it is above the upper limit
        if input(i) > upper_limit(i)
            % Saturate at upper limit.
            input(i) = upper_limit(i);

        % If it is below the lower limit
        elseif input(i) < lower_limit(i)
            % Saturate at lwoer limit.
            input(i) = lower_limit(i);
        end
    end
end