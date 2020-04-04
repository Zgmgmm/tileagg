#ifndef _CONFIG_HH
#define _CONFIG_HH

#include <string>

#include "TileAgg.hh"

#define RTSP_CLIENT_VERBOSITY_LEVEL 1

// By default, we request that the server stream its data using RTP/UDP.
// If, instead, you want to request that the server stream via RTP-over-TCP,
// change the following to True:
#define REQUEST_STREAMING_OVER_TCP False

std::string vrMediaDescription =
    "3,3,1280,720"
    " "

    // low quality
    "rtsp://localhost:8888/vr_625000_3x3_0x0x384x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_0x256x384x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_0x512x384x208.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_384x0x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_384x256x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_384x512x448x208.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_832x0x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_832x256x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3_832x512x448x208.mkv"
    " "
    "rtsp://localhost:8888/vr_625000_3x3.mkv"
    " "

    // high quality
    "rtsp://localhost:8888/vr_1500000_3x3_0x0x384x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_0x256x384x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_0x512x384x208.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_384x0x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_384x256x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_384x512x448x208.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_832x0x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_832x256x448x256.mkv"
    " "
    "rtsp://localhost:8888/vr_1500000_3x3_832x512x448x208.mkv"
    " ";

std::string papMediaDescription =
    "3,3,1840,992"
    " "
    // high quality
    "rtsp://localhost:8555/pap_10000000_3x3.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_0x0x576x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_0x320x576x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_0x640x576x352.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_1216x0x624x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_1216x320x624x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_1216x640x624x352.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_576x0x640x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_576x320x640x320.mkv"
    " "
    "rtsp://localhost:8555/pap_10000000_3x3_576x640x640x352.mkv"
    " "

    // low quality
    "rtsp://localhost:8555/pap_100000_3x3.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_0x0x576x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_0x320x576x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_0x640x576x352.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_1216x0x624x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_1216x320x624x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_1216x640x624x352.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_576x0x640x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_576x320x640x320.mkv"
    " "
    "rtsp://localhost:8555/pap_100000_3x3_576x640x640x352.mkv"
    " ";

#endif