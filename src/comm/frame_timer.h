//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef LIVOX_DRIVER_FRAME_TIMER_H_
#define LIVOX_DRIVER_FRAME_TIMER_H_

#include <chrono>
#include <cstdint>

namespace livox_ros {

// Decides when to cut a point cloud frame while the lidar is not time-synced.
//
// It runs on a monotonic clock. The system clock can jump at runtime (an
// unbacked RTC read at boot, then NTP), and a frame timer on the system clock
// that only advances by one interval per check would then publish every
// packet as its own frame until it caught up, which can take years.
//
// If the timer falls more than one interval behind (a stalled thread, or a
// burst of packets after a pause), it restarts from the current time instead
// of replaying the missed intervals.
class FrameTimer {
 public:
  using Clock = std::chrono::steady_clock;

  explicit FrameTimer(uint64_t interval_ns = 100000000) : interval_(interval_ns) {}

  void SetInterval(uint64_t interval_ns) { interval_ = std::chrono::nanoseconds(interval_ns); }

  // Returns true when a frame should be published at `now`.
  bool Tick(Clock::time_point now) {
    if (!started_) {
      last_ = now;
      started_ = true;
      return false;
    }
    if (now - last_ < interval_) {
      return false;
    }
    last_ += interval_;
    if (now - last_ >= interval_) {
      last_ = now;
    }
    return true;
  }

  bool Tick() { return Tick(Clock::now()); }

 private:
  std::chrono::nanoseconds interval_;
  Clock::time_point last_{};
  bool started_ = false;
};

}  // namespace livox_ros

#endif  // LIVOX_DRIVER_FRAME_TIMER_H_
