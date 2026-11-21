// Unit test for livox_ros::FrameTimer. No ROS or Livox SDK needed:
//   g++ -std=c++14 -I src test/test_frame_timer.cpp -o /tmp/test_frame_timer && /tmp/test_frame_timer

#include <chrono>
#include <cstdio>
#include <cstdlib>

#include "comm/frame_timer.h"

using livox_ros::FrameTimer;
using Clock = FrameTimer::Clock;
using std::chrono::milliseconds;

static int failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
      ++failures;                                                      \
    }                                                                  \
  } while (0)

// Feeds one call per packet, packets `packet_gap` apart, for `duration`, and
// returns how many frames were cut.
static int CountFrames(FrameTimer& timer, Clock::time_point& now, milliseconds packet_gap,
                       milliseconds duration) {
  int frames = 0;
  const Clock::time_point end = now + duration;
  while (now < end) {
    now += packet_gap;
    if (timer.Tick(now)) {
      ++frames;
    }
  }
  return frames;
}

static void TestTenHertzAtPacketRate() {
  FrameTimer timer(100000000);  // 100 ms
  Clock::time_point now{};
  CHECK(!timer.Tick(now));  // first call only starts the timer
  // Mid-360 sends roughly one packet per millisecond.
  const int frames = CountFrames(timer, now, milliseconds(1), milliseconds(10000));
  CHECK(frames >= 99 && frames <= 100);
}

static void TestNoBurstAfterStall() {
  // The old system-clock timer replayed every missed interval, one frame per
  // packet. After a 10 s stall (or a forward clock jump) the next frames must
  // still be ~100 ms apart, not one per packet.
  FrameTimer timer(100000000);
  Clock::time_point now{};
  timer.Tick(now);
  now += milliseconds(10000);
  CHECK(timer.Tick(now));  // one frame for the stalled period
  const int frames = CountFrames(timer, now, milliseconds(1), milliseconds(1000));
  CHECK(frames >= 9 && frames <= 10);
}

static void TestSmallLatenessKeepsCadence() {
  // A packet arriving slightly late must not shift the cadence.
  FrameTimer timer(100000000);
  Clock::time_point now{};
  timer.Tick(now);
  CHECK(timer.Tick(now + milliseconds(130)));
  CHECK(!timer.Tick(now + milliseconds(199)));
  CHECK(timer.Tick(now + milliseconds(200)));
}

static void TestIntervalChange() {
  FrameTimer timer(100000000);
  timer.SetInterval(50000000);  // 20 Hz
  Clock::time_point now{};
  timer.Tick(now);
  const int frames = CountFrames(timer, now, milliseconds(1), milliseconds(1000));
  CHECK(frames >= 19 && frames <= 20);
}

int main() {
  TestTenHertzAtPacketRate();
  TestNoBurstAfterStall();
  TestSmallLatenessKeepsCadence();
  TestIntervalChange();
  if (failures == 0) {
    std::printf("test_frame_timer: all checks passed\n");
    return EXIT_SUCCESS;
  }
  std::printf("test_frame_timer: %d check(s) failed\n", failures);
  return EXIT_FAILURE;
}
