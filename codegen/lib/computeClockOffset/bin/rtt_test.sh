#!/bin/bash

# 检查 side_ch.ko 模块是否已加载
if ! lsmod | grep -q "^side_ch"; then
    echo "side_ch.ko 未加载，尝试加载模块..."
    # 如果需要指定模块路径，可以用 insmod
    insmod /root/sdr/side_ch.ko num_eq_init=0
    # 如果模块已安装在标准模块路径下，使用 modprobe
    modprobe side_ch
    if [ $? -ne 0 ]; then
        echo "加载 side_ch.ko 失败，请检查路径或模块依赖。"
        exit 1
    fi
    echo "side_ch.ko 已成功加载。"
else
    echo "side_ch.ko 已加载。"
fi

/root/sdr/side_ch_ctl_src/computeClockOffset/bin/computeClockOffset wh1h4001
/root/sdr/side_ch_ctl_src/computeClockOffset/bin/computeClockOffset wh7hb7d738bb
#/root/sdr/side_ch_ctl_src/computeClockOffset/bin/computeClockOffset wh7h44332250
/root/sdr/side_ch_ctl_src/computeClockOffset/bin/computeClockOffset g
