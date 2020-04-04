
#ifndef _TILEAGG_HH
#define _TILEAGG_HH

#include <deque>
#include <list>

#include "BasicUsageEnvironment.hh"
#include "liveMedia.hh"

#define PLAY_TIME_UNAVAILABLE -1u

class TileAgg;
class TileBuffer;

class Frame {
 public:
  Frame(u_int8_t* data, u_int32_t size, u_int64_t rtpTimestamp = -1,
        Boolean rtpMarker = False, u_int32_t rtpSeqNum = -1);

  ~Frame();
  u_int32_t fSize;
  u_int8_t* fData;
  u_int64_t fRtpTimestamp;
  Boolean fRtpMarker;
  u_int32_t fRtpSeqNumber;
};

class TileBuffer {
 public:
  TileBuffer(TileAgg* agg, MediaSubsession* subsession = NULL);

  void queueFrame(u_int8_t* data, unsigned size, u_int64_t rtpTimestamp,
                  u_int32_t rtpSeq, Boolean rtpMarker);

  Frame* dequeueFrame();
  // return earliest play time in ticks (npt*frequency) or -1 when no frame
  // available
  int64_t curPlayTime();

  // TODO:
  // reassign MediaSubsession
  void setMediaSubsession(MediaSubsession* subsession) {}

  u_int8_t fBuffer[1024000];
  TileAgg* ourAgg;
  MediaSubsession* ourSubsession;
  std::deque<Frame*> fFrames;
  u_int32_t fNumAvailableFrames;

  u_int32_t fLastRtpTimestamp;
};

class TileAgg : public FramedSource {
 public:
  static TileAgg* createNew(UsageEnvironment& env);

 protected:
  virtual void doGetNextFrame();

 public:
  void addTile(MediaSubsession* subsession);
  void removeTile(MediaSubsession* subsession);
  typedef void onNextFrameFunc(Frame* frame);
  void setOnNextFrameCB(onNextFrameFunc* cb) { onNextFrameCB = cb; }
  Boolean startPlaying();

  //  protected:
  TileAgg(UsageEnvironment& env);

  Boolean continuePlaying();

  static void afterTileGettingFrame(void* clientData, unsigned frameSize,
                                    unsigned numTruncatedBytes,
                                    struct timeval presentationTime,
                                    unsigned durationInMicroseconds);
  static void onTileSourceClosure(void* clientData);
  Frame* aggregate();
  virtual ~TileAgg();

  //  private:
  onNextFrameFunc* onNextFrameCB;
  std::list<TileBuffer*> fTiles;
  u_int8_t fNumTiles;
  int64_t fLastPlayTime;

#define MAX_TILE_BUF_SIZE (1 << 20)  // 1MB
  u_int8_t fBuffer[MAX_TILE_BUF_SIZE];

  u_int8_t* fVPS;
  unsigned fVPSSize;
  u_int8_t* fSPS;
  unsigned fSPSSize;
  u_int8_t* fPPS;
  unsigned fPPSSize;
};
#endif