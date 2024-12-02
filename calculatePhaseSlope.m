function phaseSlopes = calculatePhaseSlope(csi)
    % 计算CSI的相位
    phase = angle(csi);
    
    phase_unwrapped = unwrap(phase, pi);
    sub_freq = 312.5e3;
    phase_x = [-28:-1, 1:28] * sub_freq;
    phase_x = phase_x.';
    phase_unwrapped = phase_unwrapped.';
    
    % 初始化相位斜率数组
    phaseSlopes = zeros(size(csi, 2), 1);
    
    %phase_y = phase_unwrapped();
    %p = polyfit(phase_x, phase_y, 1);
    for i = 1:size(csi, 2)
        phase_y = phase_unwrapped(i, :);
        p = polyfit(phase_x, phase_y, 1);
        phaseSlopes(i) = p(1);
    end

    % 计算相位斜率
    %phaseSlope = p(1);
end