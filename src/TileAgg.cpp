

#include "TileAgg.hh"

#include <assert.h>
#include <glog/logging.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <list>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "BlockingQueue.h"

// live555
#include "Base64.hh"
#include "BitVector.hh"
#include "groupsock/GroupsockHelper.hh"
#include "strDup.hh"

// SDL
#include <SDL.h>
#include <SDL_thread.h>

// FFMPEG
extern "C" {
#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
#include "libavutil/imgutils.h"
#include "libavutil/mathematics.h"
#include "libavutil/pixfmt.h"
#include "libavutil/time.h"
#include "libswscale/swscale.h"
}

using namespace std;
using namespace std::chrono;

#define TEST_OUT_DIR "test/"

void play();

static char START_CODE[] = {0, 0, 0, 1};

// Boolean timeEQ(u_int64_t a, u_int64_t b, u_int64_t tollerance = 2) {
//   return ((unsigned)std::abs(int(a - b)) < tollerance);
// }

// Boolean timeLT(u_int64_t a, u_int64_t b, u_int64_t tollerance = 2) {
//   if (timeEQ(a, b, tollerance)) return False;
//   return a < b;
// }

// Boolean timeGT(u_int64_t a, u_int64_t b, u_int64_t tollerance = 2) {
//   if (timeEQ(a, b, tollerance)) return False;
//   return a > b;
// }

Boolean timeEQ(double a, double b, double tollerance = 0.01) {
  return (std::abs(a - b) < tollerance);
}

Boolean timeLT(double a, double b, double tollerance = 0.01) {
  if (timeEQ(a, b, tollerance)) return False;
  return a < b;
}

Boolean timeGT(double a, double b, double tollerance = 0.01) {
  if (timeEQ(a, b, tollerance)) return False;
  return a > b;
}

BlockingQueue<Frame*> que;

Frame::Frame(u_int8_t* data, u_int32_t size, u_int64_t rtpTimestamp,
             Boolean rtpMarker, u_int32_t rtpSeqNum, double npt)
    : fSize(size),
      fRtpTimestamp(rtpTimestamp),
      fRtpMarker(rtpMarker),
      fRtpSeqNumber(rtpSeqNum),
      fNpt(npt) {
  fData = new u_int8_t[fSize];
  memmove(fData, data, size);
}

Frame::~Frame() { delete[] fData; }

// npt*frequency
double TileBuffer::curPlayTime() {
  // no available frames
  if (fNumAvailableFrames == 0) return PLAY_TIME_UNAVAILABLE;

  return fFrames.front()->fNpt;

  int64_t timestampOffset =
      fFrames.front()->fRtpTimestamp - ourSubsession->rtpInfo.timestamp;
  int64_t npt = ourSubsession->playStartTime() *
                    ourSubsession->rtpSource()->timestampFrequency() +
                timestampOffset;

  return npt;
  // return fFrames.front()->fRtpTimestamp - ourSubsession->rtpInfo.timestamp;
}

TileBuffer::TileBuffer(TileAgg* agg, MediaSubsession* subsession, double start)
    : ourAgg(agg),
      ourSubsession(subsession),
      fNumAvailableFrames(0),
       fLastNpt(PLAY_TIME_UNAVAILABLE) {}

void TileBuffer::queueFrame(u_int8_t* data, unsigned size,
                            u_int64_t rtpTimestamp, u_int32_t rtpSeq,
                            Boolean rtpMarker, double npt) {
  auto naluType = (data[0] & 0x7E) >> 1;
  if (naluType >= 32) {
    return;
  }

  // DEBUG: output tile queued frames
  {
    static std::map<void*, std::ofstream*> m;
    std::ofstream* os = m[this];

    if (os == NULL) {
      m[this] = os = new std::ofstream();
      char outPath[128];
      auto desc = ourSubsession->parentSession().sessionDescription();
      auto fn = strrchr(desc, '/');
      sprintf(outPath, TEST_OUT_DIR "que_%s.265", fn ? fn + 1 : desc);
      os->open(outPath);

      // write VPS/SPS/PPS once
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fVPS, ourAgg->fVPSSize);
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fSPS, ourAgg->fSPSSize);
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fPPS, ourAgg->fPPSSize);
    }
    // write file
    os->write(START_CODE, 4);
    os->write((char*)data, size);
    os->flush();
  }

  // 检查rtpTimestamp是否增加，更新可用帧数
  if (ourSubsession->rtpInfo.timestamp != 0) {
    if (fLastNpt != PLAY_TIME_UNAVAILABLE && !timeEQ(npt, fLastNpt))
      fNumAvailableFrames = fFrames.size();
  } else {  // 应在PLAY response之后才queueFrame
    LOG(ERROR) << "rtpInfo.timestamp is 0!!" << endl;
  }

  // 帧入队
  auto frame = new Frame(data, size, rtpTimestamp, rtpMarker, rtpSeq, npt);
  fFrames.push_back(frame);

  // 记录上一个rtpTimestamp
  fLastNpt = npt;
}

Frame* TileBuffer::dequeueFrame() {
  if (fNumAvailableFrames == 0) return NULL;

  fNumAvailableFrames--;
  auto frame = fFrames.front();
  fFrames.pop_front();

  // DEBUG:
  auto naluType = (frame->fData[0] & 0x7E) >> 1;
  LOG(INFO) << "[TileState]"
            << " " << ourSubsession->parentSession().sessionDescription()
            << " dequeue"
            << " naluType=" << naluType
            << " rtpSeqNumber=" << frame->fRtpSeqNumber
            << " size=" << frame->fSize << "\n";

  // DEBUG: output tile dequeued frames
  {
    auto data = frame->fData;
    auto size = frame->fSize;
    static std::map<void*, std::ofstream*> m;
    std::ofstream* os = m[this];
    if (os == NULL) {
      m[this] = os = new std::ofstream();
      char outPath[128];
      auto desc = ourSubsession->parentSession().sessionDescription();
      auto fn = strrchr(desc, '/');
      sprintf(outPath, TEST_OUT_DIR "deq_%s.265", fn ? fn + 1 : desc);
      os->open(outPath);

      // write VPS/SPS/PPS once
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fVPS, ourAgg->fVPSSize);
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fSPS, ourAgg->fSPSSize);
      os->write(START_CODE, 4);
      os->write((char*)ourAgg->fPPS, ourAgg->fPPSSize);
    }
    // write file
    os->write(START_CODE, 4);
    os->write((char*)data, size);
    os->flush();
  }

  return frame;
}

TileAgg* TileAgg::createNew(UsageEnvironment& env) { return new TileAgg(env); }

TileAgg::TileAgg(UsageEnvironment& env) : FramedSource(env) {
  fLastPlayTime = PLAY_TIME_UNAVAILABLE;
}

TileAgg::~TileAgg() {}

void TileAgg::addTile(MediaSubsession* subsession, double start) {
  auto tb = new TileBuffer(this, subsession, start);
  do {                        // parse VPS SPS PPS
    if (fVPS != NULL) break;  // init once
    u_int8_t** ps[] = {&fVPS, &fSPS, &fPPS};
    unsigned* psSize[] = {&fVPSSize, &fSPSSize, &fPPSSize};

    char const* base64List[3] = {subsession->fmtp_spropvps(),
                                 subsession->fmtp_spropsps(),
                                 subsession->fmtp_sproppps()};
    for (int i = 0; i < 3; i++) {
      auto base64 = base64List[i];
      unsigned num;
      auto record = parseSPropParameterSets(base64, num)[0];
      auto data = record.sPropBytes;
      auto size = record.sPropLength;
      *(ps[i]) = new u_int8_t[size];
      memmove(*ps[i], data, size);
      *psSize[i] = size;
    }
  } while (0);

  // increase socket receieve buffer
  fTiles.push_back(tb);
  auto gs = subsession->rtpSource()->RTPgs()->socketNum();
  auto ret = increaseReceiveBufferTo(envir(), gs, 100 * 1024 * 1024);
  LOG(INFO) << " increase receieve buffer to " << ret << "\n.";
}

void TileAgg::removeTile(MediaSubsession* subsession) {
  auto it = fTiles.begin();
  for (; it != fTiles.end(); it++) {
    auto tb = *it;
    if (tb->ourSubsession == subsession) break;
  }
  fTiles.erase(it);
}

Boolean TileAgg::startPlaying() {
  LOG(INFO) << " " << this << " start playing.\n";
  return continuePlaying();
}
Boolean TileAgg::continuePlaying() {
  for (auto& ts : fTiles) {
    if (ts->ourSubsession->rtpSource()->isCurrentlyAwaitingData()) continue;
    ts->ourSubsession->rtpSource()->getNextFrame(ts->fBuffer, MAX_TILE_BUF_SIZE,
                                                 afterTileGettingFrame, ts,
                                                 onTileSourceClosure, ts);
    // DEBUG:
    // LOG(INFO)
    //         << " " <<
    //         ts->ourSubsession->parentSession().sessionDescription()
    //         << " continue playing"
    //         << "\n";
  }
  return True;
}

void TileAgg ::afterTileGettingFrame(void* clientData, unsigned frameSize,
                                     unsigned numTruncatedBytes,
                                     struct timeval presentationTime,
                                     unsigned durationInMicroseconds) {
  auto ts = (TileBuffer*)clientData;
  auto ta = ts->ourAgg;
  auto buffer = ts->fBuffer;
  auto naluType = (buffer[0] & 0x7E) >> 1;
  u_int32_t rtpInfoSeqNum = ts->ourSubsession->rtpInfo.seqNum;
  u_int32_t rtpInfoTS = ts->ourSubsession->rtpInfo.timestamp;
  u_int32_t curPacketRTPSeqNum =
      ts->ourSubsession->rtpSource()->curPacketRTPSeqNum();
  u_int32_t curPacketRTPTimestamp =
      ts->ourSubsession->rtpSource()->curPacketRTPTimestamp();
  Boolean curPacketMarkerBit =
      ts->ourSubsession->rtpSource()->curPacketMarkerBit();
  double npt = ts->ourSubsession->getNormalPlayTime(presentationTime);

  // DEBUG: buffer overflow
  if (numTruncatedBytes > 0) {
    LOG(INFO)
        << "TileAgg::afterGettingFrame(): The input frame data was "
           "too large for our buffer.  "
        << numTruncatedBytes
        << " bytes of trailing data was dropped!  Correct this by increasing "
           "the \"bufferSize\" parameter in the \"createNew()\" call.\n";
  }

  // DEBUG: QoE
  // Assume that there's only one SSRC source (usually the case)
  // RTPReceptionStatsDB::Iterator statsIter(
  //     ts->ourSubsession->rtpSource()->receptionStatsDB());
  // RTPReceptionStats* stats = statsIter.next(True);
  // unsigned totReceivedNow = stats->totNumPacketsReceived();
  // unsigned totExpectedNow = stats->totNumPacketsExpected();
  // LOG(INFO)
  //             << " " <<
  //             ts->ourSubsession->parentSession().sessionDescription()
  //             << " packets=" << totReceivedNow << "/" << totExpectedNow <<
  //             "\n";

  // DEBUG: randomly dropping, for lossness tolerance test
  // static bool once = true;
  // if (once) {
  //   std::srand(std::time(
  //       nullptr));  // this code path runs only once in the program's
  //       lifetime
  //   once = false;
  // }
  // if (std::rand() % 1000 < 200) {
  //   LOG(INFO)
  //             << " dropping frame"
  //             << " " <<
  //             ts->ourSubsession->parentSession().sessionDescription()
  //             << " naluTyp=" << naluType << " size=" << frameSize << "\n";
  //   ta->continuePlaying();
  //   return;
  // }

  // DEBUG:
  char log[1024];
  sprintf(
      log,
      "%s npt=%5.3f naluType=%-2u size=%-5u rSeq=%-5u rTS=%-7u mark=%d seq=%-7u"
      " TS=%-12u PTS=%12ld.%06ld\n",
      ts->ourSubsession->parentSession().sessionDescription(), npt, naluType,
      frameSize, curPacketRTPSeqNum - rtpInfoSeqNum,
      curPacketRTPTimestamp - rtpInfoTS, curPacketMarkerBit, curPacketRTPSeqNum,
      curPacketRTPTimestamp, presentationTime.tv_sec, presentationTime.tv_usec);
  LOG(INFO) << log;

  ts->queueFrame(buffer, frameSize, curPacketRTPTimestamp, curPacketRTPSeqNum,
                 curPacketMarkerBit, npt);

  while (1) {
    Frame* frame = ta->aggregate();

    // no frame available
    if (frame == NULL) break;

    // HACK: send VPS SPS PPS
    // static Boolean send = True;
    // if (send) {
    //   send = False;

    //   u_int8_t buf[10240];
    //   u_int32_t frameSize = 0;
    //   Frame* frame;

    //   MediaSubsession* subsession = ta->fTiles.front()->ourSubsession;
    //   char const* base64List[3] = {subsession->fmtp_spropvps(),
    //                                subsession->fmtp_spropsps(),
    //                                subsession->fmtp_sproppps()};
    //   for (int i = 0; i < 3; i++) {
    //     auto base64 = base64List[i];
    //     unsigned num;
    //     auto records = parseSPropParameterSets(base64, num);
    //     for (unsigned j = 0; j < num; j++) {
    //       auto record = records[j];
    //       auto data = record.sPropBytes;
    //       auto size = record.sPropLength;
    //       memmove(buf + frameSize, START_CODE, 4);
    //       memmove(buf + frameSize + 4, data, size);
    //       frameSize += size + 4;
    //     }
    //   }
    //   frame = new Frame(buf, frameSize);
    //   que.push(frame);
    // }

    // que.push(frame);
  };

  // for next time
  ta->continuePlaying();
}

void play() {
  // DEBUG: flags
  static Boolean scale = False;

  static Boolean init = True;
  static AVCodecContext* avctx;
  static AVCodec* codec;
  static AVPacket* avpkt;
  static AVFrame avframe, frameYUV;
  // static int pixel_w = 1840, pixel_h = 992;
  static int pixel_w = 1280, pixel_h = 720;
  // static int pixel_w = 896, pixel_h = 512;
  static u_int8_t* buffer;

  static SDL_Window* window;
  static SDL_Renderer* renderer;
  static SDL_RendererInfo renderer_info = {0};
  static SDL_Texture* texture;
  static struct SwsContext* sws;
  static AVPixelFormat sourceFormat = AV_PIX_FMT_YUV420P;
  static AVPixelFormat targetFormat = AV_PIX_FMT_YUV420P;
  static int screen_w = pixel_w;  // 1840 / 2;
  static int screen_h = pixel_h;  // 992 / 2;
  static SDL_Rect rect{.x = 0, .y = 0, .w = screen_w, .h = screen_h};

  int ret;

  if (init) {
    init = False;

    // av init
    codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
    if (!codec) exit(AVERROR_DECODER_NOT_FOUND);

    avctx = avcodec_alloc_context3(codec);
    if (!avctx) exit(AVERROR(ENOMEM));

    if (!codec) {
      fprintf(stderr, "codec not found!");
      exit(AVERROR(EINVAL));
    }

    if ((ret = avcodec_open2(avctx, codec, NULL)) < 0) {
      exit(-1);
    }

    // init decoder
    // avctx->pix_fmt = sourceFormat;
    sws = sws_getContext(pixel_w, pixel_h, sourceFormat, screen_w, screen_h,
                         targetFormat, SWS_BICUBIC, NULL, NULL, NULL);
    buffer = (unsigned char*)av_malloc(
        av_image_get_buffer_size(AV_PIX_FMT_YUV420P, screen_w, screen_h, 1));
    av_image_fill_arrays(frameYUV.data, frameYUV.linesize, buffer,
                         AV_PIX_FMT_YUV420P, screen_w, screen_h, 1);

    // SDL init
    int flags = SDL_INIT_VIDEO | SDL_WINDOW_RESIZABLE;

    if (SDL_Init(flags)) {
      av_log(NULL, AV_LOG_FATAL, "Could not initialize SDL - %s\n",
             SDL_GetError());
      av_log(NULL, AV_LOG_FATAL, "(Did you set the DISPLAY variable?)\n");
      exit(1);
    }
    window =
        SDL_CreateWindow("tileagg", SDL_WINDOWPOS_UNDEFINED,
                         SDL_WINDOWPOS_UNDEFINED, screen_w, screen_h, flags);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "linear");
    if (window) {
      renderer = SDL_CreateRenderer(
          window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
      if (!renderer) {
        av_log(NULL, AV_LOG_WARNING,
               "Failed to initialize a hardware accelerated renderer: %s\n",
               SDL_GetError());
        renderer = SDL_CreateRenderer(window, -1, 0);
      }
      if (renderer) {
        if (!SDL_GetRendererInfo(renderer, &renderer_info))
          av_log(NULL, AV_LOG_VERBOSE, "Initialized %s renderer.\n",
                 renderer_info.name);
      }
    }
    if (!window || !renderer || !renderer_info.num_texture_formats) {
      av_log(NULL, AV_LOG_FATAL, "Failed to create window or renderer: %s",
             SDL_GetError());
      exit(-1);
    }
    SDL_EventState(SDL_SYSWMEVENT, SDL_IGNORE);
    SDL_EventState(SDL_USEREVENT, SDL_IGNORE);

    texture =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_IYUV,
                          SDL_TEXTUREACCESS_STREAMING, screen_w, screen_h);
  }

  while (1) {
    auto frame = que.pop();

    avpkt = av_packet_alloc();
    if (av_new_packet(avpkt, frame->fSize)) std::cout << "error";

    memmove(avpkt->data, frame->fData, frame->fSize);
    delete frame;

    ret = avcodec_send_packet(avctx, avpkt);
    while (ret >= 0) {
      ret = avcodec_receive_frame(avctx, &avframe);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      else if (ret < 0) {
        std::cout << "Error during decoding\n";
        exit(1);
      }
      // DEBUG: scale?
      if (scale)
        sws_scale(sws, (const uint8_t* const*)avframe.data, avframe.linesize, 0,
                  avframe.height, frameYUV.data, frameYUV.linesize);
      else
        frameYUV = avframe;
      SDL_UpdateYUVTexture(texture, &rect, frameYUV.data[0],
                           frameYUV.linesize[0], frameYUV.data[1],
                           frameYUV.linesize[1], frameYUV.data[2],
                           frameYUV.linesize[2]);
      SDL_RenderClear(renderer);
      SDL_RenderCopy(renderer, texture, NULL, &rect);
      SDL_RenderPresent(renderer);
    }
  }
}

// FIXME: merge tiles, taking VPS/SPS/PPS recorrect in concern
Frame* TileAgg::aggregate() {
  LOG(INFO) << "=========aggreating=========\n";

  double playTime;
  double earliestPlayTime = PLAY_TIME_UNAVAILABLE;
  Frame* frame;
  std::vector<Frame*> candidates;
  u_int8_t buf[1024000];
  u_int8_t naluType;
  u_int8_t* data;

  // get next play time
  for (auto ts : fTiles) {
    while (1) {
      playTime = ts->curPlayTime();
      if (playTime == PLAY_TIME_UNAVAILABLE) {
        LOG(INFO)

            << " awaiting tile "
            << ts->ourSubsession->parentSession().sessionDescription() << "\n";
        return NULL;
      }

      if (fLastPlayTime == PLAY_TIME_UNAVAILABLE ||
          timeGT(playTime, fLastPlayTime))
        break;

      // drop timeout frame
      auto naluType = (ts->fFrames.front()->fData[0] & 0x7E) >> 1;
      delete ts->dequeueFrame();

      LOG(INFO) << " drop frame "
                << ts->ourSubsession->parentSession().sessionDescription()
                << " naluType=" << naluType
                << " play_time=" << (double)playTime / 90000
                << " last_play_time=" << (double)fLastPlayTime / 90000 << "\n";
    }
    if (earliestPlayTime == PLAY_TIME_UNAVAILABLE)
      earliestPlayTime = playTime;
    else
      earliestPlayTime = std::min(playTime, earliestPlayTime);
  }

  // 取出earliestPlayTime的所有tile
  for (auto ts : fTiles) {
    while ((playTime = ts->curPlayTime()) != PLAY_TIME_UNAVAILABLE) {
      if (!timeEQ(playTime, earliestPlayTime)) {
        break;
      }

      frame = ts->dequeueFrame();
      naluType = (frame->fData[0] & 0x7E) >> 1;
      candidates.push_back(frame);
      envir() << " append Frame from "
              << ts->ourSubsession->parentSession().sessionDescription()
              << " naluType=" << naluType << " size=" << frame->fSize
              << " play_time=" << (unsigned)playTime << " npt="
              << (double)playTime / ts->ourSubsession->rtpTimestampFrequency()
              << "\n";
    }
  }

  // 选择tile
  // 1. 解析slice sgement address
  // 2. 选择相同address质量最高的tile
  // 3. FIXME: 对选择的tile按address进行排序
  std::map<u_int32_t, Frame*> outTilesMap;

  for (unsigned i = 0; i < candidates.size(); i++) {
    auto candidate = candidates[i];
    auto data = candidate->fData;
    u_int32_t sliceSegmentAddress = 0u;
    BitVector bv(data, 0, candidate->fSize * 8);
    bv.skipBits(1);                 // forbidden bit
    auto naluType = bv.getBits(6);  // NAL Unit type
    bv.skipBits(9);                 // Layer, TID
    auto firstSliceSegmentInPicFlag =
        bv.get1Bit();  // first slice segment in pic flag
    auto rapPicFlag = (16 <= naluType && naluType <= 20);
    if (rapPicFlag) bv.skipBits(1);  // no_output_of_prior_pics_flag
    bv.get_expGolomb();              // PPS id

    if (!firstSliceSegmentInPicFlag &&
        False  // FIXME: && pps->dependent_slice_segments_enabled_flag
    ) {
      bv.skipBits(1); /*dependent_slice_segment_flag*/
      break;
    }
    if (!firstSliceSegmentInPicFlag) {  // slice segment address
      sliceSegmentAddress =
          bv.getBits(9);  // FIXME: sps->bitsSliceSegmentAddress)
    }
    auto tile = outTilesMap[sliceSegmentAddress];
    if (tile != NULL) {  // FIXME: select the best version of the tile
      if (tile->fSize < candidate->fSize) {
        outTilesMap[sliceSegmentAddress] = candidate;
        delete tile;
      } else {
        delete candidate;
      }
      LOG(INFO) << " duplicate"
                << " slice_segment_address=" << sliceSegmentAddress << "\n";
    } else {
      outTilesMap[sliceSegmentAddress] = candidate;
    }

    LOG(INFO) << " aggregate"
              << " bit_index=" << bv.curBitIndex()
              << " slice_segment_address=" << sliceSegmentAddress << "\n";
  }

  std::vector<Frame*> outTiles;
  if (outTilesMap.find(0) != outTilesMap.end()) {  // first slice must be front
    outTiles.push_back(outTilesMap[0]);
    outTilesMap.erase(0);
  } else {
  }

  for (auto it = outTilesMap.begin(); it != outTilesMap.end(); ++it) {
    outTiles.push_back(it->second);
  }

  // append Frame
  // FIXME: reorder the tiles in scan order and
  // set the first_slice_in_frame_flag
  size_t tt_num = outTiles.size();
  u_int32_t tt_size = 0;

  for (auto it = outTiles.begin(); it != outTiles.end(); it++) {
    frame = *it;
    data = frame->fData;
    naluType = (data[0] & 0x7E) >> 1;
    memmove(buf + tt_size, START_CODE, 4);
    memmove(buf + tt_size + 4, frame->fData, frame->fSize);
    tt_size += 4 + frame->fSize;
    delete frame;
  }

  // clean
  candidates.clear();
  outTilesMap.clear();
  outTiles.clear();

  LOG(INFO) << " play_time=" << (double)earliestPlayTime / 90000
            << " aggregate tt_num=" << (int)tt_num
            << " tt_size=" << (int)tt_size << "\n";

  Frame* outFrame = new Frame(buf, tt_size, earliestPlayTime);
  if (!outFrame) {
    LOG(FATAL) << "new failed" << endl;
    return NULL;
  }

  fLastPlayTime = earliestPlayTime;

  // DEBUG: output aggregation
  static std::ofstream os;
  static Boolean init = True;
  if (frame != NULL) {
    if (init) {  // file init once
      init = False;
      char outPath[] = TEST_OUT_DIR "agg.265";
      os.open(outPath);

      // write VPS/SPS/PPS once
      os.write(START_CODE, 4);
      os.write((char*)fVPS, fVPSSize);
      os.write(START_CODE, 4);
      os.write((char*)fSPS, fSPSSize);
      os.write(START_CODE, 4);
      os.write((char*)fPPS, fPPSSize);
    }
    // write file
    // NOTE that frame->fData has start code
    os.write((char*)buf, tt_size);
    os.flush();
  }

  if (onNextFrameCB && outFrame) onNextFrameCB(outFrame);

  return outFrame;
}

void TileAgg::onTileSourceClosure(void* clientData) {
  TileBuffer* ts = (TileBuffer*)clientData;

  LOG(INFO) << "onTileSourceClosure " << ts << "\n";
}

void TileAgg::doGetNextFrame() {
  LOG(INFO) << " doGetNextFrame"
            << "\n";
}
