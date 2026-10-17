# 反馈时序补充记录

与 [整机日志规范](../docs/test_data_recording.md) 一起使用。此模板只定义采集语义，不向原固件自动插入代码，不声明下列字段已经存在。

每个运行保存 `run_id`、固件版本/补丁哈希、实际参数、时钟单位与回绕方式、轴、参考轨迹和功能开关。
对照至少保留同 Ki、同限幅下的 ESO 补偿开/关，并保留中断或故障运行。

| 字段 | 记录位置与含义 |
| --- | --- |
| `control_seq, control_time_us, dt_s` | 当次控制调用；`dt_s` 为真实间隔 |
| `position_seq, position_time_us, position_rad` | 角度生产者序号、源时刻和实际采用值 |
| `velocity_seq, velocity_time_us, velocity_rad_s` | 角速度生产者序号、源时刻和实际采用值 |
| `position_received_us, velocity_received_us` | 主控收到/发布样本时刻，与源时刻分开 |
| `position_age_s, velocity_age_s, feedback_age_s` | 控制时刻减去各源时刻，最后一项为两者最大值 |
| `reference_time_us, reference_position_rad, reference_velocity_rad_s, reference_acceleration_rad_s2` | 实际采用的参考及源时刻 |
| `command_nm, compensation_nm, disturbance_nm, command_word, status, flags` | 当次控制分项与最终提交电机指令；发送失败另记 |

没有源时间戳时留空，并在元数据写明 `timestamp_kind=host_receive`；不把接收时间改名为源时间。
生产者序号只在生成新样本时递增；控制任务重复读取时保留原序号和值。
若日志抽样、溢出或丢帧，保留缺口与丢失计数，不重新连续编号。
跨设备时钟未同步时，不直接相减计算年龄；注明时钟域和同步方式。

角度来源需注明编码器、INS 融合或其他估计器；角速度需注明陀螺仪或电机 RPM。
记录各自量化、滤波、安装轴变换，以及速度是否为欧拉角导数。
缺失量填空/`null`，不填零。单位转换另存脚本，保存原始记录。

记录器应复制已有计算结果，经有界缓冲异步导出；不能额外调用控制器或发送回调来“读取”结果。
该模板不指定 USART/USB 通道，宿主按实际占用和带宽选择。
