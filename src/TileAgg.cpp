

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

Frame::Frame(u_int8_t* data, u_int32_t size, double npt, Boolean rtpMarker,
             u_int32_t rtpSeqNum)
    : fSize(size), fNpt(npt), fRtpMarker(rtpMarker), fRtpSeqNumber(rtpSeqNum) {
  fData = new u_int8_t[fSize];
  memmove(fData, data, size);
}

Frame::~Frame() { delete[] fData; }

// npt*frequency
double TileBuffer::curPlayTime() {
  // no available frames
  if (fNumAvailableFrames == 0) return PLAY_TIME_UNAVAILABLE;

  return fFrames.front()->fNpt;
}

TileBuffer::TileBuffer(TileAgg* agg, MediaSubsession* subsession, double start)
    : ourAgg(agg),
      ourSubsession(subsession),
      fNumAvailableFrames(0),
      fLastNpt(PLAY_TIME_UNAVAILABLE) {}

void TileBuffer::queueFrame(u_int8_t* data, unsigned size, double npt,
                            u_int32_t rtpSeq, Boolean rtpMarker) {
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
  if (ourSubsession->rtpInfo.timestamp == 0) {
    LOG(ERROR) << "rtpInfo.timestamp is 0!!" << endl;
  }

  if (fLastNpt != PLAY_TIME_UNAVAILABLE && !timeEQ(npt, fLastNpt))
    fNumAvailableFrames = fFrames.size();
  // 帧入队
  auto frame = new Frame(data, size, npt, rtpMarker, rtpSeq);
  fFrames.push_back(frame);

  // 记录上一个npt
  fLastNpt = npt;
}

Frame* TileBuffer::dequeueFrame() {
  if (fNumAvailableFrames == 0) return NULL;

  fNumAvailableFrames--;
  auto frame = fFrames.front();
  fFrames.pop_front();

  // DEBUG:
  // auto naluType = (frame->fData[0] & 0x7E) >> 1;
  //LOG(INFO) << " " << ourSubsession->parentSession().sessionDescription()
  //          << " dequeue"
  //          << " naluType=" << naluType
  //          << " rtpSeqNumber=" << frame->fRtpSeqNumber
  //          << " size=" << frame->fSize << "\n";

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
  u_int32_t curPacketRTPSeqNum =
      ts->ourSubsession->rtpSource()->curPacketRTPSeqNum();
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
  ta->fBytesReceived += frameSize;
  LOG(INFO) << "npt=" << ta->fLastPlayTime << " receieved bytes "
            << ta->fBytesReceived << endl;
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
  sprintf(log,
          "%s npt=%5.3f naluType=%-2u size=%-5u rSeq=%-5u mark=%d seq=%-7u"
          " PTS=%12ld.%06ld\n",
          ts->ourSubsession->parentSession().sessionDescription(), npt,
          naluType, frameSize, curPacketRTPSeqNum - rtpInfoSeqNum,
          curPacketMarkerBit, curPacketRTPSeqNum, presentationTime.tv_sec,
          presentationTime.tv_usec);
  //LOG(INFO) << log;

  ts->queueFrame(buffer, frameSize, npt, curPacketRTPSeqNum,
                 curPacketMarkerBit);

  while (1) {
    Frame* frame = ta->aggregate();

    // no frame available
    if (frame == NULL) break;
    frame = NULL;
  };

  // for next time
  ta->continuePlaying();
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

  if (fTiles.empty()) return NULL;
  // get next play time
  for (auto ts : fTiles) {
    while (1) {
      playTime = ts->curPlayTime();
      if (playTime == PLAY_TIME_UNAVAILABLE) {
        //LOG(INFO) << " awaiting tile "
        //          << ts->ourSubsession->parentSession().sessionDescription()
        //         << "\n";
        return NULL;
      }

      if (fLastPlayTime == PLAY_TIME_UNAVAILABLE ||
          timeGT(playTime, fLastPlayTime))
        break;

      // drop timeout frame
      auto naluType = (ts->fFrames.front()->fData[0] & 0x7E) >> 1;
      delete ts->dequeueFrame();

      //LOG(INFO) << " drop frame "
      //          << ts->ourSubsession->parentSession().sessionDescription()
      //          << " naluType=" << naluType << " play_time=" << (double)playTime
      //          << " last_play_time=" << (double)fLastPlayTime << "\n";
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
      LOG(INFO) << " append Frame from "
              << ts->ourSubsession->parentSession().sessionDescription()
              << " naluType=" << naluType << " size=" << frame->fSize
              << " play_time=" << (unsigned)playTime << " npt="
              << playTime
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

  }

  std::vector<Frame*> outTiles;
  if (outTilesMap.find(0) != outTilesMap.end()) {  // first slice must be front
    outTiles.push_back(outTilesMap[0]);
    outTilesMap.erase(0);
  } else {
    LOG(ERROR)<<"first slice segment not found"<<endl;
    // auto convertToFistSliceSegment = [](u_int8_t* data, u_int32_t size,
    //                                     u_int32_t& outSize) {
    //   auto setUE = [](BitVector& bv, unsigned num) {
    //     int length = 1;
    //     int temp = ++num;
    //     while (temp != 1) {
    //       temp >>= 1;
    //       length += 2;
    //     }
    //     bv.putBits(0, length >> 1);
    //     bv.putBits(num, (length + 1) >> 1);
    //   };
    //   auto setSE = [setUE](BitVector& bv, unsigned num) {
    //     u_int32_t v;
    //     if (num <= 0)
    //       v = (-1 * num) << 1;
    //     else
    //       v = (num << 1) - 1;
    //     setUE(bv, v);
    //   };
    //   auto getGolomb = [](BitVector& bv, unsigned* length = NULL) {
    //     unsigned golomb;
    //     int golombBegin, golombEnd;
    //     golombBegin = bv.curBitIndex();
    //     golomb = bv.get_expGolomb();  // PPS id
    //     golombEnd = bv.curBitIndex();
    //     if (length != NULL) *length = golombEnd - golombBegin;
    //     return golomb;
    //   };
    //   auto remove_emulation_bytes = [](const u_int8_t* buffer_src,
    //                                    u_int8_t* buffer_dst,
    //                                    u_int32_t nal_size) {
    //     u_int32_t i = 0, emulation_bytes_count = 0;
    //     u_int8_t num_zero = 0;

    //     while (i < nal_size) {
    //       /*ISO 14496-10: "Within the NAL unit, any four-byte sequence that
    //         starts with 0x000003 other than the following sequences shall not
    //         occur at any byte-aligned position: 0x00000300 0x00000301 0x00000302
    //         0x00000303"
    //       */
    //       if (num_zero == 2 && buffer_src[i] == 0x03 &&
    //           i + 1 < nal_size /*next byte is readable*/
    //           && (u_int8_t)buffer_src[i + 1] < 0x04) {
    //         /*emulation code found*/
    //         num_zero = 0;
    //         emulation_bytes_count++;
    //         i++;
    //       }

    //       buffer_dst[i - emulation_bytes_count] = buffer_src[i];

    //       if (!buffer_src[i])
    //         num_zero++;
    //       else
    //         num_zero = 0;

    //       i++;
    //     }

    //     return nal_size - emulation_bytes_count;
    //   };
    //   auto add_emulation_bytes = [](const u_int8_t* buffer_src,
    //                                 u_int8_t* buffer_dst, u_int32_t nal_size) {
    //     u_int32_t i = 0, emulation_bytes_count = 0;
    //     u_int8_t num_zero = 0;

    //     while (i < nal_size) {
    //       /*ISO 14496-10: "Within the NAL unit, any four-byte sequence that
    //       starts with 0x000003 other than the following sequences shall not
    //       occur at any byte-aligned position: 0x00000300 0x00000301 0x00000302
    //       0x00000303"
    //       */
    //       if (num_zero == 2 && (u_int8_t)buffer_src[i] < 0x04) {
    //         /*add emulation code*/
    //         num_zero = 0;
    //         buffer_dst[i + emulation_bytes_count] = 0x03;
    //         emulation_bytes_count++;
    //         if (!buffer_src[i]) num_zero = 1;
    //       } else {
    //         if (!buffer_src[i])
    //           num_zero++;
    //         else
    //           num_zero = 0;
    //       }
    //       buffer_dst[i + emulation_bytes_count] = buffer_src[i];
    //       i++;
    //     }
    //     return nal_size + emulation_bytes_count;
    //   };

    //   u_int8_t* buf = new u_int8_t[size];
    //   auto input = new u_int8_t[size];  // copy
    //   auto inputSize = remove_emulation_bytes(data, input, size);
    //   BitVector res(buf, 0, size * 8);
    //   BitVector bv(input, 0, inputSize * 8);
    //   res.put1Bit(bv.get1Bit());      // forbidden bit
    //   auto naluType = bv.getBits(6);  // NAL Unit type
    //   res.putBits(naluType, 6);       // forbidden bit
    //   res.putBits(bv.getBits(9), 9);  // Layer, TID
    //   auto firstSliceSegmentInPicFlag =
    //       bv.get1Bit();  // first slice segment in pic flag
    //   res.put1Bit(1);
    //   auto rapPicFlag = (16 <= naluType && naluType <= 20);
    //   if (rapPicFlag) {
    //     res.put1Bit(bv.get1Bit());  // no_output_of_prior_pics_flag
    //   }
    //   unsigned ppsId, length;
    //   ppsId = getGolomb(bv, &length);
    //   // bv.skipBits(-length);
    //   // res.putBits(bv.getBits(1),1);
    //   res.put1Bit(1);
    //   // setUE(res, ppsId);

    //   bool dependent_slice_segments_enabled_flag =
    //       false;  // FIXME: && pps->dependent_slice_segments_enabled_flag
    //   bool dependent_slice_segment_flag = false;
    //   unsigned slice_address_length =
    //       9;  // av_ceil_log2(s->ps.sps->ctb_width * s->ps.sps->ctb_height);
    //   if (!firstSliceSegmentInPicFlag) {
    //     if (dependent_slice_segments_enabled_flag)
    //       dependent_slice_segment_flag =
    //           bv.get1Bit(); /*dependent_slice_segment_flag*/
    //   }
    //   unsigned addr;
    //   if (!firstSliceSegmentInPicFlag) {  // slice segment address
    //     addr = bv.getBits(
    //         slice_address_length);  // FIXME: sps->bitsSliceSegmentAddress)
    //   }
    //   outSize=res.curBitIndex();
    //   // outSize+= bv.numBitsRemaining();
    //   outSize = ((float)outSize + 0.99999) / 8;
    //   // shiftBits(buf, res.curBitIndex(), data, bv.curBitIndex(),
    //   //           bv.numBitsRemaining());
    //   // unsigned num_extra_slice_header_bits =
    //   //     0;  // pps->num_extra_slice_header_bits
    //   // if (!dependent_slice_segment_flag) {
    //   //   bv.skipBits(num_extra_slice_header_bits);
    //   // }

    //   // unsigned slice_type = getGolomb(bv, &length);
    //   // res.putBits(slice_type, length);

    //   // if (naluType != 19 && naluType != 20) {
    //   //   unsigned log2_max_poc_lsb = 4;  // FIXME: sps->log2_max_poc_lsb
    //   //   unsigned poc_lsb = bv.getBits(log2_max_poc_lsb);
    //   //   res.putBits(poc_lsb, 4);
    //   //   res.put1Bit(bv.get1Bit());  // short_term_ref_pic_set_sps_flag
    //   // }
    //   delete[] input;

    //   return buf;
    // };
    // auto tile = outTilesMap.begin()->second;
    // unsigned size;
    // u_int8_t* out = convertToFistSliceSegment(tile->fData, tile->fSize, size);
    // outTiles.push_back(new Frame(out, size));
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

  LOG(INFO) << " play_time=" << earliestPlayTime
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
  exit(-1);
}

void TileAgg::doGetNextFrame() {
  LOG(INFO) << " doGetNextFrame"
            << "\n";
}
