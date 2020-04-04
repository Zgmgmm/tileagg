# Tile-based Adaptive Bitrate Streaming via RTP



![view](/home/zgmgmm/Downloads/ABRTSP.jpg)



### tile分割与拼接





### VCL NAL Unit





### 内容生成

- 容器MP4?MKV?
- 1 x tiled HEVC(NxN)
- N^2 x single tile HEVC
  - 
- live555 RTP
  - liveMedia/H264or5VideoStreamFramer.cpp:1168
    - 更新rtpTimestamp条件

### Server

#### Seek如何实现?

- live555支持mkv seek

#### Server支持mkv文件中的多个track

#### Server支持不同文件在同个session

SDP



#### VPS

- vps_id(4)
- (2)
- vps_max_layers_minus1(6)
- vps_max_sub_layers_minus(3)
- vps_temporal_id_nesting_flag(1)
- Profile-Tier-Level(PTL)
- ...

#### SPS

- sps_vps_id
- sps_max_sub_layers_minus1
- sps_temporal_id_nesting_flag
- profile_tier_level
- profile_tier_level(PTL)
- sps_id
- chroma_format_idc
- **pic_width_in_luma_samples**
- **pic_width_in_luma_samples**
- conformance_window_flag
- **bit_depth_luma_minus8**
- **bit_depth_chroma_minus8**
- log2_max_pic_order_cnt_lsb_minus4?
- log2_min_luma_coding_block_size_minus3 //log2(最小CB大小)
- log2_diff_max_min_luma_coding_block_size //log2(CB大小最大最小值之差)
- ...
- VUI
  - ...
  - vui_timing_info_present_flag
    - **vui_num_units_in_tick**
    - **vui_time_scale**
    - vui_poc_proportional_to_timing_flag

#### PPS

- **tiles_enabled_flag**
  - **num_tile_columns_minus1**
  - **num_tile_rows_minus1**
  - **uniform_spacing_flag**
  - **uniform_spacing_flag**
- **pps_loop_filter_across_slices_enabled_flag**
- **deblocking_filter_control_present_flag**
  - deblocking_filter_override_enabled_flag
  - pps_deblocking_filter_disabled_flag 
  - if( !pps_deblocking_filter_disabled_flag )
    - pps_beta_offset_div2 = 0
    - pps_tc_offset_div2 = 0

#### SLICE

- first_slice_in_pic_flag
- 



### openHEVC对于tile解码有个特殊逻辑，强制first_slice_in_pic

- openHEVC/libavcodec/hevcdec.c:4245
- openHEVC/libavcodec/hevcdec.c:662









### HEVC视频信息

- 尺寸
  - SPS
    - **pic_width_in_luma_samples****
    - **pic_width_in_luma_samples**

- 码率
  - SPS
    - VUI
      - if(vui_timing_info_present_flag)
        - **vui_num_units_in_tick**
        - **vui_time_scale**
        - vui_poc_proportional_to_timing_flag



### HEVC码流涉及Tile的点

#### SPS

- **pic_width_in_luma_samples**
- **pic_width_in_luma_samples**

- log2_min_luma_coding_block_size_minus3 //log2(最小CB大小，计算)
- log2_diff_max_min_luma_coding_block_size //log2(CB大小最大最小值之差)

#### PPS

- **tiles_enabled_flag**

- **num_tile_columns_minus1**
- **num_tile_rows_minus1**
- **uniform_spacing_flag**
- **uniform_spacing_flag**
- **pps_loop_filter_across_slices_enabled_flag****
- **deblocking_filter_control_present_flag**
  - deblocking_filter_override_enabled_flag
  - pps_deblocking_filter_disabled_flag 
  - if( !pps_deblocking_filter_disabled_flag )
    - pps_beta_offset_div2 = 0
    - pps_tc_offset_div2 = 0

#### Slice

- first_slice_in_pic_flag
  - Server打包RTP会参考NAL Unit type、first_slice_in_pic_flag来确定Access Unit边界
- dependent_slice_segment_flag
- slice_segment_addr(slice_address_length)







#### LIVE555打包HEVC RTP策略

##### RTP marker bit

表明当前packet结束当前Access Unit

- True
  - end of file(source)
  - end of sequence(NAL Unit type)
  - end of bitstream(NAL Unit type)
  - first_slice_in_pic_flag(next NAL Unit)
  - usuallyBeginsAccessUnit(next NAL Unit type)
- False
  - usuallyBeginsAccessUnit
  - [default]

##### RTP timestamp

根据Access Unit来划分/增长





码率自适应策略



全图LQ下载

加FoV HQ对LQ块进行替换





FFmpeg

hwaccl 硬件加速？





### TileState



### TileSource

#### props

- regions
- bitrate
- base rtp timestamp 
- buffer
  - last rtp timestamp(tail)
  - cur play time
  - 

#### state

- INIT
  - SETUP
  - start playing
  - PLAY
  - 开始接收rtp包，response返回前无法计算npt
  - 接收到response，解析response中时间信息
    - Range(start npt)
    - rtpInfo->rtpTimestamp 
  - 将buffer中未设置npt的包补上npt
- PLAY
  - 收到一个包，确认当前Access Unit结束，条件
    - rtpMakerBit == 1
      - !EQ(rtpTimestamp, lastRtpTimestamp)
- 
- active

SETUP



live555 server 

pause后resume发送的第一个包有问题，解码帧乱码，与无pause-resume操作的播放码流相比，第一个包貌似是恢复点及后的包的错乱混合，怀疑是pause-resume操作时server暴力处理了字节流，而不是以帧为单元操作


## 程序流

- TileCrontroller
- TileAggregator
- TileDecoder
- Sensor
- Rendor

### SETUP
1. 获得`Media Description`
- 发起`DESCRIBE`
- 读取本地文件
2. 解析`Media Description`，获得以下信息
- `VPS`、`SPS`、`PPS`
  - 分块方式，如3x3、2x2
  - grid，即行高列宽，即每个tile的`x` `y` `w` `h`
- 不同版本tile的`url`、`bitrate`

3. 获取取决策因子
- `FoV`（来自Sensor）
- `Bandwidth`（来自？）
- `Strategy`

4. 决策出新的`Version`（需要的tiles及其质量)
- 对比当前的Tile状态，计算需要调整的Tile

5. 执行切换任务
- 目标切换npt（开始组合/显示新`Version`的时候）
- 发送新`PLAY`命令，删除旧`Tile`
> 尽可能**高效**且**无缝**切换，如何选择`PLAY`命令的`Range`
> 如何**替换**新旧`Tile`





1. 输入Media Description到`Controller`
- JSON
- SDP

2. 获取`Preference`




- DESCRIBE
- new session(sdp)
- session.subsession.initiate()
  - new rtpsource
- setup session
  - setupNextSubsession
  - SETUP subsession
  - add sink, sink->startPlaying()
  - setupNextSubsession
  - loop
- PLAY



url
- client
- subsession
- seek(指定range)



TileState
- TileBuffer
- start
- current timestamp


```json
{
    //framerate <- sps.vui
    //gop
    // "w":1280,
    // "h":720,
    "duration": 28.8,
    "gop": 24, // gop/framerate, or calculate on fly  
    //"rap": gop/framerate
    "sprop-vps": "QAEMAv//AWAAAAMAgAAAAwAAAwC6AAAsCQ==",
    "sprop-sps": "QgECAWAAAAMAgAAAAwAAAwC6AACgA5iAPhd5JMogEAAAPoAABhqAgA==",
    "sprop-pps": "RAHBc8JuTJA=",
    "tracks": [
        {
            "x": 0,
            "y": 0,
            "w": 1280,
            "h": 720,
            "url": "rtsp://localhost:port/file",
            "bitrate": 10000,
        },
        {
            "x": 0,
            "y": 0,
            "w": 480,
            "h": 320,
            "url": "rtsp://localhost:port/file",
            "bitrate": 10000,
        },
        {
            "x": 480,
            "y": 320,
            "w": 1280,
            "h": 720,
            "url": "rtsp://localhost:port/file",
            "bitrate": 10000,
        },
    ]
}
```