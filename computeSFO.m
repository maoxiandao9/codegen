function clockOffset = computeSFO(buffer)
    % 输入: buffer (1D array of uint16)
    % 输出: phaseSlopes (1D array of double), sfo (1D array of double)
    a = buffer;
    len_a = floor(length(a)/4)*4;
    a = a(1:len_a);

    b = reshape(a, [4, length(a)/4])';
    num_data_in_each_side_info = 2 + 56;
    num_side_info = floor(size(b,1) / num_data_in_each_side_info);

    tx_timestamp = uint32(zeros(1, num_side_info));
    rx_timestamp = uint32(zeros(1, num_side_info));
    side_info = complex(zeros(num_data_in_each_side_info, num_side_info), zeros(num_data_in_each_side_info, num_side_info));
    csi = complex(zeros(56, num_side_info), zeros(56, num_side_info));

    b = uint16(b);
    for i = 1:num_side_info
        sp = (i-1) * num_data_in_each_side_info + 1;
        ep = i * num_data_in_each_side_info;
        % 提取高位32位作为tx时间戳
        tx_timestamp(i) = uint32(b(sp,3)) + (2^16)*uint32(b(sp,4));
        % 提取低位32位作为rx时间戳
        rx_timestamp(i) = uint32(b(sp,1)) + (2^16)*uint32(b(sp,2));
        side_info(:,i) = double(typecast(b(sp:ep,1), 'int16')) + 1i .* double(typecast(b(sp:ep,2), 'int16'));
        csi(:,i) = side_info(3:58,i);
    end

    csi = [csi(29:end,:); csi(1:28,:)];
    phaseSlopes = calculatePhaseSlope(csi);
    phaseSlopes = phaseSlopes.';
    phaseSlopes_time = phaseSlopes / (2 * pi);
    sfo = UniformUnwrap(phaseSlopes_time, 25e-9);
    rx_timestamp_seconds = double(rx_timestamp) * 25 * 1e-9;
    p = polyfit(rx_timestamp_seconds, sfo, 1);
    % 固定间隔积累的时间偏移
    timeOffset = p(1); 
    freq = 20e6;
    % 固定间隔积累的时间偏移
    timeOffsetScaled = timeOffset / freq;  % 直接计算缩放后的时间偏移
    clockOffset = freq / (1 - freq * timeOffsetScaled) - freq;
end
