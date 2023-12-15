#pragma once

#include <map>
#include <string>
#include <iostream>
#include <atomic>
#include <mutex>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace aivot::profiler
{
    namespace {
        inline double to_seconds(const boost::posix_time::time_duration& d) {
            return (double)d.total_microseconds() / 1000000.0;
        }

        struct DataIntVal
        {
            std::string name_;
            unsigned long int value_;
        };

        struct SortIntByValue
        {
            bool operator()(const DataIntVal& a, const DataIntVal& b) const
            {
                return a.value_ > b.value_;
            }
        };

        struct DataDoubleVal
        {
            std::string name_;
            double value_;
        };

        struct SortDoubleByValue
        {
            bool operator()(const DataDoubleVal& a, const DataDoubleVal& b) const
            {
                return a.value_ > b.value_;
            }
        };
    }

    class SpinLock {
    public:
        SpinLock() noexcept = default;
        ~SpinLock() noexcept = default;

        SpinLock(const SpinLock &) = delete;
        SpinLock &operator=(const SpinLock &) = delete;

        void lock() noexcept {
            for (;;) {
                if (!m_lock.exchange(true, std::memory_order_acquire)) {
                    return;
                }

                while (m_lock.load(std::memory_order_relaxed)) {
                    // Issue X86 PAUSE or ARM YIELD instruction to reduce contention between HT cores
                    __builtin_ia32_pause();
                }
            }
        }

        bool try_lock() noexcept {
            // First do a relaxed load to check if lock is free in order to prevent
            // unnecessary cache misses if someone does while(!try_lock())
            return !m_lock.load(std::memory_order_relaxed) &&
                   !m_lock.exchange(true, std::memory_order_acquire);
        }

        void unlock() noexcept {
            m_lock.store(false, std::memory_order_release);
        }

    private:
        std::atomic<bool> m_lock = { false };
    };

    class Profiler
    {
    public:
        Profiler(const Profiler &) = delete;
        Profiler& operator=(const Profiler &) = delete;

        explicit Profiler(bool printOnDestroy = false, bool autoStart = false)
                : m_isRunning(false), m_printOnDestroy(printOnDestroy)
        {
            if (autoStart) {
                start();
            }
        }

        ~Profiler() {
            if (m_printOnDestroy && !m_data.empty()) {
                status();
            }
        }

        static Profiler& Default() {
            static Profiler instance(false, false);
            return instance;
        }

        void start() {
            std::lock_guard<SpinLock> lock(m_lock);
            if (!m_isRunning) {
                m_threadInfo.set();
                m_isRunning = true;
            }
        }

        void stop() {
            std::lock_guard<SpinLock> lock(m_lock);
            if (m_isRunning) {
                m_threadInfo.update();
                m_isRunning = false;
            }
        }

        void clear() {
            std::lock_guard<SpinLock> lock(m_lock);
            m_data.clear();
            m_threadInfo = TimeInfo();
            if (m_isRunning) {
                m_threadInfo.set();
            }
        }

        void console() {
            std::stringstream ss;
            ss << std::endl;
            status(ss, true);
        }

        void status(std::ostream& out = std::cout, bool merge = true) {
            stop();
            std::lock_guard<SpinLock> lock(m_lock);
            m_printOnDestroy = false;

            out << std::endl;
            out << " *** Profiling statistics. Total counted time : " << to_seconds(m_threadInfo.total) << " seconds" << std::endl;

            if (merge) {
                PerThread combined;
                for (auto & it : m_data)
                {
                    for (const std::pair<const std::string, unsigned long int>& event : it.second.events)
                        combined.events[event.first] += event.second;
                    for (const std::pair<const std::string, AvgInfo>& iavg : it.second.avg)
                    {
                        combined.avg[iavg.first].total += iavg.second.total;
                        combined.avg[iavg.first].totalSqr += iavg.second.totalSqr;
                        combined.avg[iavg.first].parts += iavg.second.parts;
                    }
                    for (const std::pair<const std::string, TimeInfo>& itm : it.second.time)
                    {
                        TimeInfo& tc = combined.time[itm.first];
                        tc.total = tc.total + itm.second.total;
                        tc.parts = tc.parts + itm.second.parts;
                        if (tc.shortest > itm.second.shortest)
                            tc.shortest = itm.second.shortest;
                        if (tc.longest < itm.second.longest)
                            tc.longest = itm.second.longest;
                    }
                }
                printThreadInfo(out, combined);
            } else {
                for (auto & it : m_data) {
                    out << "Thread " << it.first << ":" << std::endl;
                    printThreadInfo(out, it.second);
                }
            }
        }

        [[nodiscard]] bool running() const {
            return m_isRunning;
        }

        void begin(const std::string& name) {
            std::lock_guard<SpinLock> lock(m_lock);
            m_data[boost::this_thread::get_id()].time[name].set();
        }

        void end(const std::string& name) {
            std::lock_guard<SpinLock> lock(m_lock);
            m_data[boost::this_thread::get_id()].time[name].update();
        }

        void event(const std::string& name, unsigned int times = 1) {
            std::lock_guard<SpinLock> lock(m_lock);
            m_data[boost::this_thread::get_id()].events[name] += times;
        }

        void average(const std::string& name, const double value) {
            std::lock_guard<SpinLock> lock(m_lock);
            AvgInfo& a = m_data[boost::this_thread::get_id()].avg[name];
            a.total += value;
            a.totalSqr += value * value;
            a.parts++;
        }
    private:

        struct TimeInfo {
            TimeInfo()
                    : parts(0)
                    , total(0, 0, 0, 0)
                    , shortest(boost::posix_time::pos_infin)
                    , longest(boost::posix_time::neg_infin)

            { }

            unsigned long int parts;
            boost::posix_time::ptime start;

            boost::posix_time::time_duration total;
            boost::posix_time::time_duration shortest;
            boost::posix_time::time_duration longest;

            void set() {
                start = boost::posix_time::microsec_clock::universal_time();
            }

            void update() {
                const boost::posix_time::time_duration& dt = boost::posix_time::microsec_clock::universal_time() - start;
                if (dt > longest)
                    longest = dt;
                if (dt < shortest)
                    shortest = dt;
                total = total + dt;
                ++parts;
            }
        };

        struct AvgInfo {
            /** \brief The sum of the values to average */
            double total;
            /** \brief The sub of squares of the values to average */
            double totalSqr;
            /** \brief Number of times a value was added to this structure */
            unsigned long int parts;
        };

        struct PerThread {
            /** \brief The stored events */
            std::map<std::string, unsigned long int> events;
            /** \brief The stored averages */
            std::map<std::string, AvgInfo> avg;
            /** \brief The amount of time spent in various places */
            std::map<std::string, TimeInfo> time;
        };

        void printThreadInfo(std::ostream& out, const PerThread& data) const {
            double total = to_seconds(m_threadInfo.total);

            std::vector<DataIntVal> events;
            for (const std::pair<const std::string, unsigned long>& event : data.events) {
                DataIntVal next = { event.first, event.second };
                events.push_back(next);
            }
            std::sort(events.begin(), events.end(), SortIntByValue());

            if (!events.empty()) {
                out << "Events:" << std::endl;
            }
            for (const DataIntVal& event : events) {
                out << event.name_ << ": " << event.value_ << std::endl;
            }

            std::vector<DataDoubleVal> avg;
            for (const std::pair<const std::string, AvgInfo>& ia : data.avg) {
                DataDoubleVal next = { ia.first, ia.second.total / (double)ia.second.parts };
                avg.push_back(next);
            }
            std::sort(avg.begin(), avg.end(), SortDoubleByValue());

            if (!avg.empty()) {
                out << "Averages:" << std::endl;
            }
            for (const DataDoubleVal& average : avg) {
                const AvgInfo& a = data.avg.find(average.name_)->second;
                out << average.name_ << ": " << average.value_ << " (stddev = "
                    << sqrt(fabs(a.totalSqr - (double)a.parts * average.value_ * average.value_) / ((double)a.parts - 1.)) << ")"
                    << std::endl;
            }

            std::vector<DataDoubleVal> time;

            for (const std::pair<const std::string, TimeInfo>& itm : data.time) {
                DataDoubleVal next = { itm.first, to_seconds(itm.second.total) };
                time.push_back(next);
            }

            std::sort(time.begin(), time.end(), SortDoubleByValue());
            if (!time.empty()) {
                out << "Blocks of time:" << std::endl;
            }

            double unaccounted = total;
            for (DataDoubleVal& time_block : time) {
                const TimeInfo& d = data.time.find(time_block.name_)->second;

                double t_s = to_seconds(d.shortest);
                double t_l = to_seconds(d.longest);
                out << time_block.name_ << ": " << time_block.value_ << "s (" << (100.0 * time_block.value_ / total) << "%), ["
                    << t_s << "s --> " << t_l << " s], " << d.parts << " parts";
                if (d.parts > 0) {
                    double pavg = to_seconds(d.total) / (double) d.parts;
                    out << ", " << pavg << " s on average";
                    if (pavg < 1.0) {
                        out << " (" << 1.0 / pavg << " /s)";
                    }
                }
                out << std::endl;
                unaccounted -= time_block.value_;
            }
            // if we do not appear to have counted time multiple times, print the unaccounted time too
            if (unaccounted >= 0.0) {
                out << "Unaccounted time : " << unaccounted;
                if (total > 0.0) {
                    out << " (" << (100.0 * unaccounted / total) << " %)";
                }
                out << std::endl;
            }
            out << std::endl;
        }

        SpinLock m_lock;
        std::map<boost::thread::id, PerThread> m_data;
        TimeInfo m_threadInfo;

        bool m_isRunning;
        bool m_printOnDestroy;
    };

    class ScopedBlock {
    public:
        ScopedBlock(const ScopedBlock&) = delete;
        ScopedBlock& operator=(const ScopedBlock&) = delete;

        explicit ScopedBlock(const std::string &name, Profiler &prof = Profiler::Default())
                : m_name(name), m_prof(prof)
        {
            m_prof.begin(name);
        }

        ~ScopedBlock()
        {
            m_prof.end(m_name);
        }

    private:
        std::string m_name;
        Profiler &m_prof;
    };

    class ScopedStart {
    public:
        ScopedStart(const ScopedStart&) = delete;
        ScopedStart& operator=(const ScopedStart&) = delete;

        explicit ScopedStart(Profiler &prof)
                : m_prof(prof), m_wasRunning(m_prof.running())
        {
            if (!m_wasRunning) {
                m_prof.start();
            }
        }

        ~ScopedStart()
        {
            if (!m_wasRunning) {
                m_prof.stop();
            }
        }

    private:
        Profiler& m_prof;
        bool m_wasRunning;
    };
}
