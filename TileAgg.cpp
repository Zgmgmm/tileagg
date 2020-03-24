

#include "TileAgg.hh"

#include <assert.h>

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <vector>

#include "Base64.hh"
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

#define INBUF_SIZE 4096
#define AUDIO_INBUF_SIZE 20480
#define AUDIO_REFILL_THRESH 4096

#define PLAY_TIME_UNAVAILABLE -1

using namespace std;
using namespace chrono;

void play();

template <typename T>
class BlockingQueue {
 public:
  BlockingQueue(u_int32_t maxSize = 102400) : fMaxSize(maxSize) {}

  void push(const T& item, long timeout = 0) {
    {
      std::unique_lock<std::mutex> lck(fMtx);
      while (fQue.size() >= fMaxSize) {
        fCondNotFull.wait(lck);
      }
      fQue.push_back(item);
    }
    fCondNotEmpty.notify_all();
  }

  T pop(long timeout = 0) {
    static int cnt = 0;
    T item;
    {
      cnt++;
      std::unique_lock<std::mutex> lck(fMtx);
      while (fQue.empty()) {
        fCondNotEmpty.wait(lck);
      }
      item = fQue.front();
      fQue.pop_front();
    }
    fCondNotFull.notify_all();
    return item;
  }

 private:
  u_int32_t fMaxSize;
  std::deque<T> fQue;
  std::mutex fMtx;                                      // 全局互斥锁.
  std::condition_variable fCondNotFull, fCondNotEmpty;  // 全局条件
};

BlockingQueue<Frame*> que;
static char startCode[] = {0, 0, 0, 1};

Frame::Frame(u_int8_t* data, u_int32_t size, u_int64_t rtpTimestamp,
             Boolean rtpMarker, u_int32_t rtpSeqNum)
    : fSize(size),
      fRtpTimestamp(rtpTimestamp),
      fRtpMarker(rtpMarker),
      fRtpSeqNumber(rtpSeqNum) {
  fData = new u_int8_t[fSize];
  memmove(fData, data, size);
}

Frame::~Frame() { delete[] fData; }

Boolean rtpTimestampEQ(u_int64_t a, u_int64_t b, int tollerance = 10) {
  return (std::abs(int(a - b)) < tollerance);
}

// npt*frequency
int64_t TileState::curPlayTime() {
  // no available frames
  if (fNumAvailableFrames == 0) return PLAY_TIME_UNAVAILABLE;

  return fFrames.front()->fRtpTimestamp - ourSubsession->rtpInfo.timestamp;
}

TileState::TileState(TileAgg* agg, MediaSubsession* subsession)
    : ourAgg(agg), ourSubsession(subsession), fNumAvailableFrames(0) {}

void TileState::queueFrame(u_int8_t* data, unsigned size,
                           u_int64_t rtpTimestamp, u_int32_t rtpSeq,
                           Boolean rtpMarker) {
  // FIXME: hack, set first_slice_segment_in_pic_flag to 0
  // 1. get slice_segment_address
  // 2. set first_slice_segment_in_pic_flag(9 bits in pap.265)
  // int addr = (data[2] & 0x7F);
  // addr |= (data[3] & 0xC0) >> 6;
  // Boolean isFirst = (this == ourAgg->fTiles[0]);
  // auto naluType = (data[0] & 0x7E) >> 1;
  // if (!isFirst)
  //   if (naluType < 32) data[2] = data[2] & 0x7F;

  auto naluType = (data[0] & 0x7E) >> 1;
  if (naluType >= 32) {
    // ourAgg->envir() << "[TileState]"
    //                 << "queueFrame drop frame"
    //                 << " naluType=" << naluType
    //                 << " rtpTimestamp=" << int(rtpTimestamp)
    //                 << " rtpSeq=" << int(rtpSeq) << " rtpMarker=" << rtpMarker
    //                 << "\n";
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
      sprintf(outPath, "que_%s", desc);
      os->open(outPath);

      // write VPS/SPS/PPS once
      os->write(startCode, 4);
      os->write((char*)ourAgg->fVPS, ourAgg->fVPSSize);
      os->write(startCode, 4);
      os->write((char*)ourAgg->fSPS, ourAgg->fSPSSize);
      os->write(startCode, 4);
      os->write((char*)ourAgg->fPPS, ourAgg->fPPSSize);
    }
    // write file
    os->write(startCode, 4);
    os->write((char*)data, size);
    os->flush();
  }

  // hack，同个Access Unit的rtp包可能有不同timestamp，由发送端rouding导致
  if (!fFrames.empty() && rtpTimestamp != fFrames.back()->fRtpTimestamp &&
      rtpTimestampEQ(rtpTimestamp, fFrames.back()->fRtpTimestamp)) {
    ourAgg->envir() << "[TileState]"
                    << " recorrect timestamp " << (int)rtpTimestamp << "->"
                    << (int)fFrames.back()->fRtpTimestamp << "\n";
    rtpTimestamp = fFrames.back()->fRtpTimestamp;
  }

  if (ourSubsession->rtpInfo.timestamp != 0) {
    // udpate number of available frames
    if (rtpMarker) {  // mark the end of last Access Unit
      fNumAvailableFrames = fFrames.size() + 1;
    } else {  // start of next Access Unit
      if (!fFrames.empty() &&
          !rtpTimestampEQ(rtpTimestamp, fFrames.back()->fRtpTimestamp))
        fNumAvailableFrames = fFrames.size();
    }
  }

  auto frame = new Frame(data, size, rtpTimestamp, rtpMarker, rtpSeq);
  fFrames.push_back(frame);

  if (fLastRtpTimestamp > rtpTimestamp &&
      !rtpTimestampEQ(fLastRtpTimestamp, rtpTimestamp)) {
    ourAgg->envir() << "[TileState]"
                    << " error order " << (int)fLastRtpTimestamp << ">"
                    << (int)rtpTimestamp;
  }

  fLastRtpTimestamp = rtpTimestamp;
}

Frame* TileState::dequeueFrame() {
  if (fNumAvailableFrames == 0) return NULL;
  fNumAvailableFrames--;
  auto frame = fFrames.front();
  fFrames.pop_front();

  // DEBUG:
  auto naluType = (frame->fData[0] & 0x7E) >> 1;
  ourAgg->envir() << "[TileState]"
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
      sprintf(outPath, "deq_%s", desc);
      os->open(outPath);

      // write VPS/SPS/PPS once
      os->write(startCode, 4);
      os->write((char*)ourAgg->fVPS, ourAgg->fVPSSize);
      os->write(startCode, 4);
      os->write((char*)ourAgg->fSPS, ourAgg->fSPSSize);
      os->write(startCode, 4);
      os->write((char*)ourAgg->fPPS, ourAgg->fPPSSize);
    }
    // write file
    os->write(startCode, 4);
    os->write((char*)data, size);
    os->flush();
  }

  return frame;
}

TileAgg* TileAgg::createNew(UsageEnvironment& env) { return new TileAgg(env); }

TileAgg::TileAgg(UsageEnvironment& env)
    : FramedSource(env), fLastPlayTime(PLAY_TIME_UNAVAILABLE) {}

TileAgg::~TileAgg() {
  // TODO:
}

void TileAgg::addTileSubsession(MediaSubsession* subsession) {
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

  TileState* ts;
  ts = new TileState(this, subsession);
  fTiles[fNumTiles++] = ts;
  auto gs = subsession->rtpSource()->RTPgs()->socketNum();
  auto ret = increaseReceiveBufferTo(envir(), gs, 100 * 1024 * 1024);
  envir() << "[TileAgg]"
          << " increase receieve buffer to " << ret << "\n.";
}

Boolean TileAgg::startPlaying() {
  envir() << "[TileAgg]"
          << " " << this << " start playing.\n";
  return continuePlaying();
}
Boolean TileAgg::continuePlaying() {
  // envir() << "[DEBUG]"
  //         << "[TileAgg]"
  //         << " continue playing"
  //         << " num_tiles=" << fNumTiles << "\n";
  for (int i = 0; i < fNumTiles; i++) {
    TileState* ts = fTiles[i];
    if (ts->ourSubsession->rtpSource()->isCurrentlyAwaitingData()) continue;
    // envir() << "[TileAgg]"
    //         << " " <<
    //         ts->ourSubsession->parentSession().sessionDescription()
    //         << " continue playing"
    //         << "\n";
    ts->ourSubsession->rtpSource()->getNextFrame(ts->fBuffer, MAX_TILE_BUF_SIZE,
                                                 afterTileGettingFrame, ts,
                                                 onTileSourceClosure, ts);
  }
  return True;
}

void TileAgg ::afterTileGettingFrame(void* clientData, unsigned frameSize,
                                     unsigned numTruncatedBytes,
                                     struct timeval presentationTime,
                                     unsigned durationInMicroseconds) {
  TileState* ts = (TileState*)clientData;
  auto ta = ts->ourAgg;
  auto buffer = ts->fBuffer;
  auto naluType = (buffer[0] & 0x7E) >> 1;

  // DEBUG: buffer overflow
  if (numTruncatedBytes > 0) {
    ta->envir()
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
  // ta->envir() << "[TileAgg]"
  //             << " " << ts->ourSubsession->parentSession().sessionDescription()
  //             << " packets=" << totReceivedNow << "/" << totExpectedNow << "\n";

  // DEBUG: randomly dropping, for lossness tolerance test
  // static bool once = true;
  // if (once) {
  //   std::srand(std::time(
  //       nullptr));  // this code path runs only once in the program's
  //       lifetime
  //   once = false;
  // }
  // if (std::rand() % 100 < 10) {
  //   ta->envir() << "[TileAgg]"
  //                      << " dropping frame"
  //                      <<
  //                      ts->ourSubsession->parentSession().sessionDescription()
  //                      << " size=" << frameSize << "\n";
  //   goto continue_playing;
  // }
 

  u_int32_t rtpInfoSeqNum = ts->ourSubsession->rtpInfo.seqNum;
  u_int32_t rtpInfoTS = ts->ourSubsession->rtpInfo.timestamp;
  u_int32_t curPacketRTPSeqNum =
      ts->ourSubsession->rtpSource()->curPacketRTPSeqNum();
  u_int32_t curPacketRTPTimestamp =
      ts->ourSubsession->rtpSource()->curPacketRTPTimestamp();
  Boolean curPacketMarkerBit =
      ts->ourSubsession->rtpSource()->curPacketMarkerBit();

  char log[1024];
  sprintf(log,
          "%s naluType=%-2u size=%-5u rSeq=%-5u rTS=%-7u mark=%d seq=%-7u"
          " TS=%-12u PTS=%12ld.%06ld\n",
          ts->ourSubsession->parentSession().sessionDescription(), naluType,
          frameSize, curPacketRTPSeqNum - rtpInfoSeqNum,
          curPacketRTPTimestamp - rtpInfoTS, curPacketMarkerBit,
          curPacketRTPSeqNum, curPacketRTPTimestamp, presentationTime.tv_sec,
          presentationTime.tv_usec);
  ta->envir() << "[TileAgg] " << log;

  // DEBUG: skip non-VCL
  // if (naluType < 32)
  ts->queueFrame(buffer, frameSize, curPacketRTPTimestamp, curPacketRTPSeqNum,
                 curPacketMarkerBit);

  while (1) {
    Frame* frame = ta->aggregate();

    // TODO: decode and play
    if (frame == NULL) break;

    void play(TileAgg * agg, Frame * frame);

    // HACK: send VPS SPS PPS
    static Boolean send = True;
    if (send) {
      send = False;

      u_int8_t buf[10240];
      u_int32_t frameSize = 0;
      Frame* frame;

      MediaSubsession* subsession = ta->fTiles[0]->ourSubsession;
      char const* base64List[3] = {subsession->fmtp_spropvps(),
                                   subsession->fmtp_spropsps(),
                                   subsession->fmtp_sproppps()};
      for (int i = 0; i < 3; i++) {
        auto base64 = base64List[i];
        unsigned num;
        auto records = parseSPropParameterSets(base64, num);
        for (unsigned j = 0; j < num; j++) {
          auto record = records[j];
          auto data = record.sPropBytes;
          auto size = record.sPropLength;
          memmove(buf + frameSize, startCode, 4);
          memmove(buf + frameSize + 4, data, size);
          frameSize += size + 4;
        }
      }
      frame = new Frame(buf, frameSize);
      que.push(frame);
    }

    que.push(frame);

    // delete frame;
  };
  // }

  // for next time
  ta->continuePlaying();
}

void play() {
  // DEBUG: flags
  static Boolean scale = True;

  static Boolean init = True;
  static AVCodecContext* avctx;
  static AVCodec* codec;
  static AVPacket* avpkt;
  static AVFrame avframe, frameYUV;
  static int pixel_w = 1840, pixel_h = 992;
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

  static u_int8_t startCode[] = {0, 0, 0, 1};
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
  envir() << "=========aggreating=========\n";

  int64_t playTime;
  int64_t earliestPlayTime = PLAY_TIME_UNAVAILABLE;
  TileState* ts;
  Frame* frame;
  std::vector<Frame*> out;
  u_int8_t buf[1024000];
  u_int8_t naluType;
  u_int8_t* data;

  // get next play time
  for (int i = 0; i < fNumTiles; i++) {
    ts = fTiles[i];
    do {
      playTime = ts->curPlayTime();
      if (playTime == PLAY_TIME_UNAVAILABLE) {
        envir()
            << "[TileAgg]"
            << " awaiting tile "
            << fTiles[i]->ourSubsession->parentSession().sessionDescription()
            << "\n";
        return NULL;
      }
      // drop timeout frame
      if (playTime <= fLastPlayTime) {
        delete ts->dequeueFrame();
        envir() << "[TileAgg]"
                << " drop frame " << i << " play_time=" << (unsigned)playTime
                << "\n";
        continue;
      }
      if (earliestPlayTime == PLAY_TIME_UNAVAILABLE)
        earliestPlayTime = playTime;
      else
        earliestPlayTime = std::min(playTime, earliestPlayTime);
    } while (earliestPlayTime == PLAY_TIME_UNAVAILABLE);
  }

  //将时间范围内的tile聚合，server发送的rtpTime由于rounding误差可能导致采样时间间隔有略微变化（个位数）
  for (int i = 0; i < fNumTiles; i++) {
    ts = fTiles[i];

    while ((playTime = ts->curPlayTime()) != PLAY_TIME_UNAVAILABLE) {
      if (std::abs((int)(playTime - earliestPlayTime)) > 10) {
        break;
      }

      frame = ts->dequeueFrame();
      naluType = (frame->fData[0] & 0x7E) >> 1;
      out.push_back(frame);
      envir() << "[TileAgg]"
              << " append Frame from "
              << ts->ourSubsession->parentSession().sessionDescription()
              << " type=" << naluType << " size=" << frame->fSize
              << " play_time=" << (unsigned)playTime << " npt="
              << (double)playTime / ts->ourSubsession->rtpTimestampFrequency()
              << "\n";
    }
  }

  // 对tile进行排序
  for (auto i = 0; i < out.size(); i++) {
    auto tile = out[i];
    auto first = (tile->fData[2] & 0x80);  // first_slice_segment_in_pic_flag
    if (first && i != 0) {
      swap(out[i], out[0]);
      break;
    }
  }

  // append Frame
  // FIXME: reorder the tiles in scan order and
  // set the first_slice_in_frame_flag
  size_t tt_num = out.size();
  u_int32_t tt_size = 0;

  if (tt_num < fNumTiles) {
    envir() << "[TileAgg]"
            << " aggregate tt_num=" << (int)tt_num << "\n";
  }

  for (auto it = out.begin(); it != out.end(); it++) {
    frame = *it;
    data = frame->fData;
    naluType = (data[0] & 0x7E) >> 1;
    memmove(buf + tt_size, startCode, 4);
    memmove(buf + tt_size + 4, frame->fData, frame->fSize);
    tt_size += 4 + frame->fSize;
    delete frame;
  }

  Frame* outFrame = new Frame(buf, tt_size, earliestPlayTime);

  fLastPlayTime = earliestPlayTime;

  // DEBUG: output aggregation
  static std::ofstream os;
  static Boolean init = True;
  if (frame != NULL) {
    if (init) {  // file init once
      init = False;
      char outPath[] = "agg.265";
      os.open(outPath);

      // write VPS/SPS/PPS once
      os.write(startCode, 4);
      os.write((char*)fVPS, fVPSSize);
      os.write(startCode, 4);
      os.write((char*)fSPS, fSPSSize);
      os.write(startCode, 4);
      os.write((char*)fPPS, fPPSSize);
    }
    // write file
    // NOTE that frame->fData has start code
    os.write((char*)buf, tt_size);
    os.flush();
  }

  return outFrame;
}

void TileAgg::onTileSourceClosure(void* clientData) {
  TileState* ts = (TileState*)clientData;

  ts->ourAgg->envir() << "onTileSourceClosure " << ts << "\n";
}

void TileAgg::doGetNextFrame() {
  envir() << "[TileAgg]"
          << " doGetNextFrame"
          << "\n";
}
