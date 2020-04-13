#include <signal.h>
#include <time.h>

#include <fstream>
#include <thread>
#include <vector>

#include "BlockingQueue.h"
#include "OurUsageEnvironment.hh"
#include "TileAgg.hh"
#include "ToString.hh"
#include "config.hh"
#include "glm/glm.hpp"
#include "glog/logging.h"
#include "json/json.h"
#include "liveMedia.hh"
#include "misc.hh"
#include "rtsp.hh"

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
using namespace google;

class VideoTrackDesc;
class VideoDesc;
class TileState;
class ourRTSPClient;

/* const */
static char START_CODE[] = {0, 0, 0, 1};

/* global context */
char* progName;
TaskScheduler* scheduler;
UsageEnvironment* env;
char eventLoopWatchVariable;
VideoDesc* videoDesc;
unsigned numRows, numCols;
vector<unsigned> rowHeights, colWidths;
TileState* tileStates[16][16];
unsigned rtspClientCount;
TileAgg* ta;

/* function declarations */
void signalHandlerShutdown(int sig);

// The main streaming routine (for each "rtsp://" URL):
RTSPClient* openURL(char const* rtspURL);

// RTSP 'response handlers':
void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode,
                           char* resultString);
void continueAfterSETUP(RTSPClient* rtspClient, int resultCode,
                        char* resultString);
void continueAfterPLAY(RTSPClient* rtspClient, int resultCode,
                       char* resultString);
void continueAfterPAUSE(RTSPClient* rtspClient, int resultCode,
                        char* resultString);

void shutdownStream(RTSPClient* rtspClient, int exitCode = 0);

// Other event handler functions:
void subsessionAfterPlaying(
    void* clientData);  // called when a stream's subsession (e.g., audio or
                        // video substream) ends
void subsessionByeHandler(void* clientData, char const* reason);
// called when a RTCP "BYE" is received for a subsession
void streamTimerHandler(void* clientData);
// called at the end of a stream's expected duration (if the stream has not
// already signaled its end using a RTCP "BYE")

/* function implements */
void signalHandlerShutdown(int sig) {
  LOG(INFO) << "signal shutdown" << endl;
  exit(0);
}

RTSPClient* openURL(char const* rtspURL) {
  // Begin by creating a "RTSPClient" object.  Note that there is a separate
  // "RTSPClient" object for each stream that we wish to receive (even if more
  // than stream uses the same "rtsp://" URL).
  RTSPClient* rtspClient = ourRTSPClient::createNew(
      *env, rtspURL, RTSP_CLIENT_VERBOSITY_LEVEL, progName);
  if (rtspClient == NULL) {
    *env << "Failed to create a RTSP client for URL \"" << rtspURL
         << "\": " << env->getResultMsg() << "\n";
    return NULL;
  }

  ++rtspClientCount;

  // Next, send a RTSP "DESCRIBE" command, to get a SDP description for the
  // stream. Note that this command - like all RTSP commands - is sent
  // asynchronously; we do not block, waiting for a response. Instead, the
  // following function call returns immediately, and we handle the RTSP
  // response later, from within the event loop:
  rtspClient->sendDescribeCommand(continueAfterDESCRIBE);

  return rtspClient;
}

// Implementation of the RTSP 'response handlers':

void continueAfterDESCRIBE(RTSPClient* rtspClient, int resultCode,
                           char* resultString) {
  do {
    UsageEnvironment& env = rtspClient->envir();                 // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to get a SDP description: " << resultString
          << "\n";
      delete[] resultString;
      break;
    }

    char* const sdpDescription = resultString;
    env << *rtspClient << "Got a SDP description:\n" << sdpDescription << "\n";

    // Create a media session object from this SDP description:
    scs.session = MediaSession::createNew(env, sdpDescription);

    delete[] sdpDescription;  // because we don't need it anymore
    if (scs.session == NULL) {
      env << *rtspClient
          << "Failed to create a MediaSession object from the SDP description: "
          << env.getResultMsg() << "\n";
      break;
    } else if (!scs.session->hasSubsessions()) {
      env << *rtspClient
          << "This session has no media subsessions (i.e., no \"m=\" lines)\n";
      break;
    }

    // Then, create and set up our data source objects for the session.  We do
    // this by iterating over the session's 'subsessions', calling
    // "MediaSubsession::initiate()", and then sending a RTSP "SETUP" command,
    // on each one. (Each 'subsession' will have its own data source.)
    scs.iter = new MediaSubsessionIterator(*scs.session);
    scs.subsession = scs.iter->next();  // only one now

    // setup subsession
    MediaSubsession* subsession = scs.subsession;
    if (subsession == NULL) {
      LOG(ERROR) << "no subsession " << scs.session->sessionName() << endl;
      break;
    }
    if (!subsession->initiate()) {
      LOG(ERROR) << "Failed to initiate the " << subsession->mediumName() << " "
                 << env.getResultMsg() << endl;
      break;
    }
    // Continue setting up this subsession, by sending a RTSP "SETUP" command:
    rtspClient->sendSetupCommand(*subsession, continueAfterSETUP, False,
                                 REQUEST_STREAMING_OVER_TCP);

    return;
  } while (0);

  // An unrecoverable error occurred with this stream.
  shutdownStream(rtspClient);
}

void continueAfterSETUP(RTSPClient* rtspClient, int resultCode,
                        char* resultString) {
  UsageEnvironment& env = rtspClient->envir();                 // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs;  // alias
  do {
    if (resultCode != 0) {
      env << *rtspClient << "Failed to set up the \"" << *scs.subsession
          << "\" subsession: " << resultString << "\n";
      break;
    }

    env << *rtspClient << "Set up the \"" << *scs.subsession
        << "\" subsession (";
    if (scs.subsession->rtcpIsMuxed()) {
      env << "client port " << scs.subsession->clientPortNum();
    } else {
      env << "client ports " << scs.subsession->clientPortNum() << "-"
          << scs.subsession->clientPortNum() + 1;
    }
    env << ")\n";

    // Having successfully setup the subsession, create a data sink for it, and
    // call "startPlaying()" on it. (This will prepare the data sink to receive
    // data; the actual flow of data from the client won't start happening until
    // later, after we've sent a RTSP "PLAY" command.

    env << *rtspClient << "Created a data sink for the \"" << *scs.subsession
        << "\" subsession\n";
    // Also set a handler to be called if a RTCP "BYE" arrives for this
    // subsession:
    if (scs.subsession->rtcpInstance() != NULL) {
      scs.subsession->rtcpInstance()->setByeWithReasonHandler(
          subsessionByeHandler, scs.subsession);
    }
  } while (0);
  delete[] resultString;

  // We've finished setting up all of the subsessions.  Now, send a RTSP "PLAY"
  // command to start the streaming:
  if (scs.session->absStartTime() != NULL) {
    // Special case: The stream is indexed by 'absolute' time, so send an
    // appropriate "PLAY" command:
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY,
                                scs.session->absStartTime(),
                                scs.session->absEndTime());
  } else {
    scs.duration = scs.session->playEndTime() - scs.session->playStartTime();
    double start;
    if (ta->fLastPlayTime == PLAY_TIME_UNAVAILABLE)
      start = 0;
    else {
      // start = (ta->fLastPlayTime + 1.2) / 90000;
      start = ta->fLastPlayTime + 0.4;
      int t = start / 0.4;
      if (t * 0.4 < start) t++;
      start = t * 0.4;
    }
    scs.start = start;
    rtspClient->sendPlayCommand(*scs.session, continueAfterPLAY, start, -1);
  }
}

void continueAfterPLAY(RTSPClient* rtspClient, int resultCode,
                       char* resultString) {
  Boolean success = False;

  do {
    UsageEnvironment& env = rtspClient->envir();                 // alias
    StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs;  // alias

    if (resultCode != 0) {
      env << *rtspClient << "Failed to start playing session: " << resultString
          << "\n";
      break;
    }

    // Set a timer to be handled at the end of the stream's expected duration
    // (if the stream does not already signal its end using a RTCP "BYE").  This
    // is optional.  If, instead, you want to keep the stream active - e.g., so
    // you can later 'seek' back within it and do another RTSP "PLAY" - then you
    // can omit this code. (Alternatively, if you don't want to receive the
    // entire stream, you could set this timer for some shorter value.) if
    // (scs.duration > 0) {
    //   unsigned const delaySlop = 2; // number of seconds extra to delay,
    //   after the stream's expected duration.  (This is optional.) scs.duration
    //   += delaySlop; unsigned uSecsToDelay = (unsigned)(scs.duration*1000000);
    //   scs.streamTimerTask =
    //   env.taskScheduler().scheduleDelayedTask(uSecsToDelay,
    //   (TaskFunc*)streamTimerHandler, rtspClient);
    // }

    env << *rtspClient << "Started playing session";
    if (scs.duration > 0) {
      env << " (for up to " << scs.duration << " seconds)";
    }
    env << "...\n";

    success = True;

    ta->addTile(scs.subsession, scs.start);
    ta->startPlaying();
    scs.subsession->miscPtr = rtspClient;
  } while (0);
  delete[] resultString;

  if (!success) {
    // An unrecoverable error occurred with this stream.
    shutdownStream(rtspClient);
  } else {
    // static Boolean d = True;
    // if (!d) return;
    // d = False;
    // UsageEnvironment& env = rtspClient->envir();  // alias
    //                                               // pause after seconds
    // void abr(void* clientData);
    // env.taskScheduler().scheduleDelayedTask( 2.4* 1e6, (TaskFunc*)abr, NULL);
    // StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs;
    // void pauseFunc(void* clientData);
    // env.taskScheduler().scheduleDelayedTask(2.5 * 1e6, (TaskFunc*)pauseFunc,
    //                                         rtspClient);
  };
}

// Implementation of the other event handlers:

void subsessionAfterPlaying(void* clientData) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)(subsession->miscPtr);

  // Begin by closing this subsession's stream:
  Medium::close(subsession->sink);
  subsession->sink = NULL;

  // Next, check whether *all* subsessions' streams have now been closed:
  MediaSession& session = subsession->parentSession();
  MediaSubsessionIterator iter(session);
  while ((subsession = iter.next()) != NULL) {
    if (subsession->sink != NULL) return;  // this subsession is still active
  }

  // All subsessions' streams have now been closed, so shutdown the client:
  shutdownStream(rtspClient);
}

void subsessionByeHandler(void* clientData, char const* reason) {
  MediaSubsession* subsession = (MediaSubsession*)clientData;
  RTSPClient* rtspClient = (RTSPClient*)subsession->miscPtr;
  UsageEnvironment& env = rtspClient->envir();  // alias

  env << *rtspClient << "Received RTCP \"BYE\"";
  if (reason != NULL) {
    env << " (reason:\"" << reason << "\")";
    delete[](char*) reason;
  }
  env << " on \"" << *subsession << "\" subsession\n";

  // Now act as if the subsession had closed:
  subsessionAfterPlaying(subsession);
}

void streamTimerHandler(void* clientData) {
  ourRTSPClient* rtspClient = (ourRTSPClient*)clientData;
  StreamClientState& scs = rtspClient->scs;  // alias

  scs.streamTimerTask = NULL;

  // Shut down the stream:
  shutdownStream(rtspClient);
}

void shutdownStream(RTSPClient* rtspClient, int exitCode) {
  UsageEnvironment& env = rtspClient->envir();                 // alias
  StreamClientState& scs = ((ourRTSPClient*)rtspClient)->scs;  // alias

  // First, check whether any subsessions have still to be closed:
  if (scs.session != NULL) {
    MediaSubsessionIterator iter(*scs.session);
    MediaSubsession* subsession;

    while ((subsession = iter.next()) != NULL) {
      if (subsession->sink != NULL) {
        Medium::close(subsession->sink);
        subsession->sink = NULL;

        if (subsession->rtcpInstance() != NULL) {
          subsession->rtcpInstance()->setByeHandler(
              NULL, NULL);  // in case the server sends a RTCP "BYE" while
                            // handling "TEARDOWN"
        }
      }
    }

    // Send a RTSP "TEARDOWN" command, to tell the server to shutdown the
    // stream. Don't bother handling the response to the "TEARDOWN".
    rtspClient->sendTeardownCommand(*scs.session, NULL);
  }

  env << *rtspClient << "Closing the stream.\n";
  Medium::close(rtspClient);
  // Note that this will also cause this stream's "StreamClientState" structure
  // to get reclaimed.

  if (--rtspClientCount == 0) {
    // The final stream has ended, so exit the application now.
    // (Of course, if you're embedding this code into your own application, you
    // might want to comment this out, and replace it with
    // "eventLoopWatchVariable = 1;", so that we leave the LIVE555 event loop,
    // and continue running "main()".)
    exit(exitCode);
  }
}

class VideoTrackDesc : public ToString {
 public:
  string toString() {
    char s[256];
    sprintf(s, "VideoTrackDesc(%s)", url);
    return string(s);
  }

  char* url;
  unsigned bitrate;
  Rect region;
};

class VideoDesc {
 public:
  double duration;
  double rap;
  unsigned gop;
  char* baseUrl;
  char *spropVPS, *spropSPS, *spropPPS;
  vector<VideoTrackDesc*> tracks;
};

VideoDesc* getVideoDesc(const char* file) {
  Json::Value root;
  std::ifstream ifs;
  ifs.open(file);

  Json::CharReaderBuilder builder;
  builder["collectComments"] = true;
  JSONCPP_STRING errs;
  if (!parseFromStream(builder, ifs, &root, &errs)) {
    LOG(WARNING) << errs << std::endl;
    return NULL;
  }

  auto videoDesc = new VideoDesc();
  videoDesc->duration = root["duration"].asDouble();
  videoDesc->gop = root["gop"].asUInt();
  videoDesc->spropVPS = strDup(root["sprop-vps"].asCString());
  videoDesc->spropSPS = strDup(root["sprop-sps"].asCString());
  videoDesc->spropPPS = strDup(root["sprop-pps"].asCString());
  videoDesc->baseUrl = strDup(root["base-url"].asCString());

  auto tracks = root["tracks"];
  for (auto track : tracks) {
    auto trackDesc = new VideoTrackDesc();
    trackDesc->url = strDup(track["url"].asCString());
    trackDesc->bitrate = track["bitrate"].asUInt();
    trackDesc->region = Rect(track["x"].asUInt(), track["y"].asUInt(),
                             track["w"].asUInt(), track["h"].asUInt());

    LOG(INFO) << *trackDesc << endl;

    videoDesc->tracks.push_back(trackDesc);
  }

  return videoDesc;
};

class TileState : public ToString {
 public:
  string toString() {
    char s[256];
    sprintf(s, "TileState(region=%s)", region.toString().c_str());
    return string(s);
  }

  RTSPClient* rtspClient;
  VideoTrackDesc* videoTrackDesc;
  Rect region;
  bool visible;
  clock_t lastTimeVisible;
  bool predictedVisible;
};

void funcShutdown(void* clientData) {
  auto ts = (TileState*)clientData;
  ourRTSPClient* client = (ourRTSPClient*)ts->rtspClient;

  if (client == NULL) {
    LOG(INFO) << "tile has no track" << endl;
    return;
  }

  LOG(INFO) << "shutdown " << ts->videoTrackDesc->toString() << " "
            << rtspClientCount << endl;

  ts->rtspClient = NULL;

  auto& scs = client->scs;
  ta->removeTile(scs.subsession);
  shutdownStream(client);
};

void funcSetup(void* clientData) {
  auto ts = (TileState*)clientData;
  ourRTSPClient* client = (ourRTSPClient*)ts->rtspClient;
  if (ts->videoTrackDesc == NULL) {
    LOG(ERROR) << "tile has no track" << endl;
    return;
  }
  if (client != NULL) {
    LOG(INFO) << "tile has client!" << endl;
    return;
  }

  char url[512];
  strcpy(url, videoDesc->baseUrl);
  strcat(url, ts->videoTrackDesc->url);

  LOG(INFO) << "setup " << url << " " << rtspClientCount << endl;

  ts->rtspClient = openURL(url);
  // if (strcmp(url, "rtsp://localhost:8888/vr_1500000_3x3_0x0x384x256.mkv"))
  //   env->taskScheduler().scheduleDelayedTask(
  //       2 * 4 * 1e6, (TaskFunc*)funcShutdown, ts->rtspClient);
};

static long abrDuration = 0.5 * 1e6;

void abr(void* clientData) {
  extern vector<glm::vec2> visibleVertices;
  extern vector<glm::vec2> predictedVisibleVertices;
  int w = 1280, h = 720;

  auto findTile = [tileStates, numRows, numCols](int x, int y) {
    for (unsigned i = 0; i < numRows; ++i) {
      for (unsigned j = 0; j < numCols; ++j) {
        auto ts = tileStates[i][j];
        if (ts->region.contains(x, y)) {
          return ts;
        }
      }
    }
  };

  // init visible and predict state of tiles
  for (unsigned i = 0; i < numRows; ++i)
    for (unsigned j = 0; j < numCols; ++j) {
      tileStates[i][j]->visible = false;
      tileStates[i][j]->predictedVisible = false;
    }

  // visible
  // for (auto& v : visibleVertices) {
  //   int x = w * v.s, y = h * v.t;
  //   auto ts = findTile(x, y);
  //   if (ts == NULL) {
  //     LOG(ERROR) << "counld find a tile!" << endl;
  //     continue;
  //   }
  //   ts->visible = true;
  //   ts->lastTimeVisible = clock();
  // }
  // // predict
  for (auto& v : predictedVisibleVertices) {
    int x = w * v.s, y = h * v.t;
    auto ts = findTile(x, y);
    if (ts == NULL) {
      LOG(ERROR) << "counld find a tile!" << endl;
      continue;
    }
    ts->predictedVisible = true;
  }

  for (unsigned i = 0; i < numRows; ++i) {
    for (unsigned j = 0; j < numCols; ++j) {
      // if (i == 0 && j == 0) continue;
      auto& ts = tileStates[i][j];
      if (ts->visible) {
        if (ts->rtspClient == NULL) {
          LOG(WARNING) << "tile visible but not active" << *ts << endl;
          env->taskScheduler().scheduleDelayedTask(0 * 1e6,
                                                   (TaskFunc*)funcSetup, ts);
        }
      } else if (ts->predictedVisible) {
        if (ts->rtspClient == NULL)
          env->taskScheduler().scheduleDelayedTask(0 * 1e6,
                                                   (TaskFunc*)funcSetup, ts);
      } else {  // invisible now and in the future
        if (ts->rtspClient != NULL) {
          auto dt = clock() - ts->lastTimeVisible;
          if ((float)dt / CLOCKS_PER_SEC > 0.5) {  // invisible for 0.5s
            env->taskScheduler().scheduleDelayedTask(
                0 * 1e6, (TaskFunc*)funcShutdown, ts);
          }
        }
      }
    }
  }

  // char log[1024];
  // sprintf(log, "x=%d y=%d left=%d right=%d top=%d bottom=%d", x, y, left,
  // right,
  //         top, bottom);
  // for (auto ts : roi) {
  //   sprintf(log + strlen(log), " %s", ts->toString().c_str());
  // }
  // LOG(INFO) << log << endl;

  env->taskScheduler().scheduleDelayedTask(abrDuration, (TaskFunc*)abr, NULL);
};

BlockingQueue<Frame*> frameQue;
BlockingQueue<AVFrame*> picQue;

void onNextFrame(Frame* frame) {
  LOG(INFO) << "onNextFrame " << frame->fRtpTimestamp << " " << frame->fSize
            << endl;
  frameQue.push(frame);
}

void decoderThreadFunc() {
  // FFmpeg
  AVCodecContext* avctx;
  AVCodec* codec;
  AVPacket* avpkt;
  AVFrame* avframe;
  struct SwsContext* sws;
  int srcW, srcH, dstW, dstH;
  AVPixelFormat srcFmt;
  AVPixelFormat dstFmt = AV_PIX_FMT_RGB24;

  int ret;

  // init HEVC decoder
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

  // allocation
  avframe = av_frame_alloc();
  avpkt = av_packet_alloc();

  while (1) {
    auto frame = frameQue.pop();

    if (av_new_packet(avpkt, frame->fSize)) LOG(ERROR) << "error";

    memmove(avpkt->data, frame->fData, frame->fSize);
    delete frame;

    ret = avcodec_send_packet(avctx, avpkt);
    while (ret >= 0) {
      ret = avcodec_receive_frame(avctx, avframe);
      if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        break;
      else if (ret < 0) {
        LOG(ERROR) << "Error during decoding\n";
        exit(1);
      }

      srcW = avctx->width, srcH = avctx->height;
      dstW = srcW, dstH = srcH;
      srcFmt = avctx->pix_fmt;

      sws = sws_getCachedContext(sws, srcW, srcH, srcFmt, dstW, dstH, dstFmt,
                                 SWS_BICUBIC, NULL, NULL, NULL);
      // scale
      auto dstFrame = av_frame_alloc();
      dstFrame->width = dstW, dstFrame->height = dstH,
      dstFrame->format = dstFmt;
      auto buffer = (unsigned char*)av_malloc(
          av_image_get_buffer_size(dstFmt, dstW, dstH, 1));
      av_image_fill_arrays(dstFrame->data, dstFrame->linesize, buffer, dstFmt,
                           dstW, dstH, 1);
      sws_scale(sws, (const uint8_t* const*)avframe->data, avframe->linesize, 0,
                avframe->height, dstFrame->data, dstFrame->linesize);

      // void updateTexture(void* data, int width, int height);
      // updateTexture(dstFrame->data[0], dstW, dstH);
      picQue.push(dstFrame);

      av_frame_unref(avframe);
    }

    av_packet_unref(avpkt);
  }

  // free memory
  avcodec_free_context(&avctx);
  av_frame_free(&avframe);
  av_packet_free(&avpkt);
  sws_freeContext(sws);
}

int main(int argc, char** argv) {
  progName = argv[0];

  // We need at least one argument:
  if (argc < 2) {
    LOG(FATAL) << "Usage: " << progName << " <rtsp-url-1> ... <rtsp-url-N>\n"
               << "\t(where each <rtsp-url-i> is a \"rtsp://\" URL)\n";
    return 1;
  }

  // setting up singal handler
  signal(SIGHUP, signalHandlerShutdown);
  signal(SIGUSR1, signalHandlerShutdown);

  /* init glog */
  {
    google::InitGoogleLogging(progName);
    /* set flags */
    // FLAGS_logtostderr = 1; // only stderr
    FLAGS_log_dir = "./log";    // to file
    FLAGS_alsologtostderr = 0;  // file and stderr
    FLAGS_stderrthreshold = 1;  // INFO
    FLAGS_minloglevel = 0;      // INFO/WARNNING/ERROR/FATAL
  }

  /* setting up our usage environment */
  scheduler = BasicTaskScheduler::createNew();
  env = OurUsageEnvironment::createNew(*scheduler);

  // get a VideoDesc from json
  videoDesc = getVideoDesc(argv[1]);
  if (videoDesc == NULL) {
    return -1;
  }

  // parse VideoDesc

  // parse tile rows and col from sprops
  numRows = 3;
  numCols = 3;
  rowHeights = {256, 256, 208};
  colWidths = {384, 448, 448};

  // grid
  unsigned x, y, w, h;
  x = y = w = h = 0;
  for (unsigned i = 0; i < numRows; ++i) {
    x = 0;
    h = rowHeights[i];
    for (unsigned j = 0; j < numCols; ++j) {
      w = colWidths[j];
      auto ts = new TileState();
      ts->region = Rect(x, y, w, h);
      tileStates[i][j] = ts;
      x += w;
    }
    y += h;
  }

  // DEBUG:
  for (unsigned i = 0; i < numRows; ++i) {
    for (unsigned j = 0; j < numCols; ++j) {
      LOG(INFO) << *tileStates[i][j];
    }
  }

  // find available tracks for each tile
  for (unsigned i = 0; i < numRows; ++i) {
    for (unsigned j = 0; j < numCols; ++j) {
      // if (i > 1 || j > 1) continue;
      auto ts = tileStates[i][j];
      for (auto trackDesc : videoDesc->tracks) {
        if (!trackDesc->region.equals(ts->region)) continue;
        ts->videoTrackDesc = trackDesc;
        break;
      }
    }
  }

  // init TileAgg
  ta = TileAgg::createNew(*env);
  ta->setOnNextFrameCB(onNextFrame);

  // init decoder
  // send VPS/SPS/PPS
  {
    u_int8_t buf[1024];
    u_int32_t frameSize = 0;
    Frame* frame;

    char const* base64List[3] = {videoDesc->spropVPS, videoDesc->spropSPS,
                                 videoDesc->spropPPS};
    for (int i = 0; i < 3; i++) {
      auto base64 = base64List[i];
      unsigned num;
      auto records = parseSPropParameterSets(base64, num);
      for (unsigned j = 0; j < num; j++) {
        auto record = records[j];
        auto data = record.sPropBytes;
        auto size = record.sPropLength;
        memmove(buf + frameSize, START_CODE, 4);
        memmove(buf + frameSize + 4, data, size);
        frameSize += size + 4;
      }
    }
    frame = new Frame(buf, frameSize);
    frameQue.push(frame);
  }
  void startRendor();
  startRendor();

  thread decoderThread(decoderThreadFunc);

  // setting up playing
  for (auto trackDesc : videoDesc->tracks) {
    if (trackDesc->region.w == 1280) {
      auto ts = new TileState();
      ts->region = trackDesc->region;
      ts->videoTrackDesc = trackDesc;
      env->taskScheduler().scheduleDelayedTask(0, (TaskFunc*)funcSetup, ts);
      break;
    }
  }

  // for (unsigned i = 0; i < numRows; ++i) {
  //   for (unsigned j = 0; j < numCols; ++j) {
  //     auto ts = tileStates[i][j];
  //     if (ts->videoTrackDesc == NULL) continue;

  //     env->taskScheduler().scheduleDelayedTask(0, (TaskFunc*)funcSetup, ts);
  //     // openURL(url);
  //   }
  // }

  // // mocking setup/shutdown tiles
  // for (unsigned k = 1; k < 10; k++) {
  //   for (unsigned i = 0; i < numRows; ++i) {
  //     for (unsigned j = 0; j < numCols; ++j) {
  //       if (i == 0) continue;
  //       auto ts = tileStates[j][i];
  //       if (ts->videoTrackDesc == NULL) continue;
  //       env->taskScheduler().scheduleDelayedTask((k * 12) * 1e6,
  //                                                (TaskFunc*)funcShutdown,
  //                                                ts);
  //       env->taskScheduler().scheduleDelayedTask((6 + k * 12) * 1e6,
  //                                                (TaskFunc*)funcSetup, ts);
  //     }
  //   }
  // }

  // FoV adaptive bitrate
  env->taskScheduler().scheduleDelayedTask(abrDuration, (TaskFunc*)abr, NULL);

  eventLoopWatchVariable = 0;
  // All subsequent activity takes place within the event loop:
  env->taskScheduler().doEventLoop(&eventLoopWatchVariable);
  // This function call does not return, unless, at some point in time,
  // "eventLoopWatchVariable" gets set to something non-zero.

  // reclaim the (small) memory used by these objects
  env->reclaim();
  env = NULL;
  delete scheduler;
  scheduler = NULL;
}