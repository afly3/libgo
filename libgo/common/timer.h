#pragma once
#include <chrono>
#include "clock.h"
#include "ts_queue.h"
#include "spinlock.h"
#include "util.h"

namespace co
{

template <typename F>
class Timer
{
    struct Element : public TSQueueHook, public RefObject {
        F cb_;
        LFLock active_;
        void* volatile slot_;

        inline void init(F const& cb) {
            cb_ = cb;
            if (active_.try_lock()) active_.unlock();
            slot_ = nullptr;
        }

        inline void call() noexcept {
            std::unique_lock<LFLock> lock(active_, std::defer_lock);
            if (!lock.try_lock()) return ;
            slot_ = nullptr;
            cb_();
        }

        // 删除时可能正在切换齿轮, 无法立即回收, 不过没关系, 下一次切换齿轮的时候会回收
        inline bool cancel() {
            if (!active_.try_lock()) return false;
            if (slot_)
                ((TSQueue<Element>*)slot_)->erase(this);
            return true;
        }

        // 切换齿轮时, 如果isValid == false, 则直接DecrementRef
        inline bool isValid() { return !active_.is_lock(); }
    };
    typedef TSQueue<Element> Slot;
    typedef TSQueue<Element> Pool;

public:
    struct TimerId
    {
        TimerId() : timer_(nullptr) {}
        TimerId(Element* elem, Timer* timer) : elem_(elem), timer_(timer) {}

        explicit operator bool() const {
            return elem_ && timer_;
        }

        bool StopTimer() { return timer_ ? timer_->StopTimer(elem_) : true; }

    private:
        SharedPtr<Element> elem_;
        Timer* timer_;
    };

public:
    template <typename Rep, typename Period>
    explicit Timer(std::chrono::duration<Rep, Period> precision);
    ~Timer();

    void SetPoolSize(int min, int max);

    TimerId StartTimer(FastSteadyClock::duration dur, F const& cb);
    
    bool StopTimer(TimerId const& id);

    void ThreadRun();

private:
    Element* NewElement();

    void DeleteElement(Element*);

private:
    int minPoolSize_ = 0;
    int maxPoolSize_ = 0;
    Pool pool_;

    // 起始时间
    FastSteadyClock::time_point begin_;

    // 精度
    FastSteadyClock::duration precision_;

    // 齿轮
    std::vector<std::vector<Slot>> slots_;
    std::vector<volatile int> points_;

    // 齿轮上一次转动时的时间点
    FastSteadyClock::time_point last_;

    static const int s_gear1 = 4;
    static const int s_gear1Shift = 2;

    static const int s_gear = 64;
    static const int s_gearShift = 6;
};

template <typename F>
template <typename Rep, typename Period>
Timer::Timer(std::chrono::duration<Rep, Period> precision)
{
    begin_ = FastSteadyClock::now();
    last_ = begin_;
    precision_ = std::chrono::duration_cast<FastSteadyClock::duration>(precision);
    int level = 1;
    for (auto timeRange = precision * s_gear1; timeRange <= std::chrono::years(4); level++) {
        timeRange *= s_gear;
    }
    points_.resize(level);
    slots_.resize(level);
    slots_[0].resize(s_gear1);
    for (int i = 1; i < level; i++)
        slots_[i].resize(s_gear);
}

template <typename F>
void Timer::SetPoolSize(int min, int max)
{
    minPoolSize_ = min;
    maxPoolSize_ = max;
}

template <typename F>
Timer<F>::TimerId Timer::StartTimer(FastSteadyClock::duration dur, F const& cb)
{
    Element* element = NewElement();
    element->init(cb);
    uint64_t durVal = (FastSteadyClock::now() + dur - begin_).count() / precision_.count();
    int level = 0;
    int offset = 0;
    uint64_t durVal1 = durVal >> s_gear1Shift;
    if (durVal1 == 0) {
        offset = durVal & (s_gear1 - 1);
    } else {
        ++level;
        uint64_t durValN = durVal1 >> s_gearShift;
        while (durValN > 0 && level + 1 < slots_.size()) {
            ++level;
            durVal1 = durValN;
            durValN = durVal1 >> s_gearShift;
        }
        offset = durVal1 & (s_gear - 1);
    }

    TimerId timerId(element, this);
    auto & wheel = slots_[level];
    wheel[(points_[level] + offset) % wheel.size()].push(element);
    return timerId;
}
    
template <typename F>
bool Timer::StopTimer(TimerId const& id)
{
    return id ? id->cancel() : true;
}

template <typename F>
void Timer::ThreadRun()
{
    std::vector<int> destPoints(points_.size());
    for (;;) {
        auto now = FastSteadyClock::now();

        uint64_t durVal = (now - begin_).count() / precision_.count();
        destPoints[0] = durVal & (s_gear1 - 1);
        durVal = durVal >> s_gear1Shift;

        for (std::size_t i = 1; i < destPoints.size(); ++i) {
            destPoints[i] = durVal & (s_gear - 1);
            durVal = durVal >> s_gearShift;
        }
    }
}

} // namespace co
