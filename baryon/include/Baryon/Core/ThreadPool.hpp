/*
 * Baryon - A custom physics engine
 * Copyright (C) 2026 Kerim Aslan
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <atomic>
#include <algorithm>

namespace Baryon::Core {

/**
 * @class ThreadPool
 * @brief Fiziksel adaların eşzamanlı ve deterministik çözümü için hafif std::jthread iş parçacığı havuzu.
 * @details C++20/C++23 std::jthread ile otomatik durdurma ve katılma (join) desteği sağlar.
 *          Adaların paralelleştirilmesinde görev sırasını deterministik kılar.
 */
class ThreadPool {
private:
    std::vector<std::jthread> mWorkers;
    std::queue<std::function<void()>> mTasks;
    std::mutex mQueueMutex;
    std::condition_variable mCondition;
    std::condition_variable mWaitCondition;
    std::atomic<bool> mStop{false};
    std::atomic<size_t> mActiveTasks{0};

public:
    /**
     * @brief Belirtilen iş parçacığı sayısıyla havuzu başlatır.
     * @param threads Çalıştırılacak iş parçacığı sayısı (0 verilirse std::thread::hardware_concurrency()).
     */
    explicit ThreadPool(size_t threads = 0) {
        resize(threads);
    }

    ~ThreadPool() {
        stop();
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    /**
     * @brief Havuzun iş parçacığı sayısını yeniden boyutlandırır.
     * @param threads Yeni iş parçacığı sayısı.
     */
    void resize(size_t threads) {
        stop();
        if (threads == 0) {
            threads = std::thread::hardware_concurrency();
            if (threads == 0) threads = 4;
        }
        mStop.store(false);
        mWorkers.clear();
        mWorkers.reserve(threads);

        for (size_t i = 0; i < threads; ++i) {
            mWorkers.emplace_back([this](std::stop_token st) {
                while (!st.stop_requested() && !mStop.load()) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(mQueueMutex);
                        mCondition.wait(lock, [this, &st]() {
                            return mStop.load() || st.stop_requested() || !mTasks.empty();
                        });

                        if ((mStop.load() || st.stop_requested()) && mTasks.empty()) {
                            return;
                        }

                        if (mTasks.empty()) continue;
                        task = std::move(mTasks.front());
                        mTasks.pop();
                    }

                    task();

                    {
                        std::unique_lock<std::mutex> lock(mQueueMutex);
                        --mActiveTasks;
                        if (mActiveTasks.load() == 0 && mTasks.empty()) {
                            mWaitCondition.notify_all();
                        }
                    }
                }
            });
        }
    }

    /**
     * @brief Havuzdaki tüm iş parçacıklarını güvenle durdurur.
     */
    void stop() {
        mStop.store(true);
        mCondition.notify_all();
        mWorkers.clear(); // std::jthread yıkımında otomatik stop_requested ve join yapılır
        {
            std::unique_lock<std::mutex> lock(mQueueMutex);
            while (!mTasks.empty()) mTasks.pop();
            mActiveTasks.store(0);
        }
    }

    /**
     * @brief Havuzdaki aktif iş parçacığı sayısını döndürür.
     */
    [[nodiscard]] size_t getThreadCount() const {
        return mWorkers.size();
    }

    /**
     * @brief Kuyruğa tek bir iş ekler.
     */
    void enqueue(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(mQueueMutex);
            mTasks.push(std::move(task));
            ++mActiveTasks;
        }
        mCondition.notify_one();
    }

    /**
     * @brief Kuyruktaki tüm işlerin ve çalışan iş parçacıklarının tamamlanmasını bekler.
     */
    void waitAll() {
        std::unique_lock<std::mutex> lock(mQueueMutex);
        mWaitCondition.wait(lock, [this]() {
            return mTasks.empty() && mActiveTasks.load() == 0;
        });
    }

    /**
     * @brief N adet bağımsız görevi deterministik bloklar halinde iş parçacıklarına dağıtır ve tamamlanmasını bekler.
     * @tparam Func Callable tipi void(size_t index)
     * @param count Toplam görev adedi.
     * @param func Çağrılacak fonksiyon nesnesi.
     */
    template <typename Func>
    void parallelFor(size_t count, Func&& func) {
        if (count == 0) return;
        if (mWorkers.size() <= 1 || count == 1) {
            // Tek iş parçacığı veya tek görev: sıralı çalıştır
            for (size_t i = 0; i < count; ++i) {
                func(i);
            }
            return;
        }

        size_t numThreads = mWorkers.size();
        size_t batchSize = (count + numThreads - 1) / numThreads;

        for (size_t b = 0; b < numThreads; ++b) {
            size_t start = b * batchSize;
            if (start >= count) break;
            size_t end = std::min(start + batchSize, count);

            enqueue([start, end, &func]() {
                for (size_t i = start; i < end; ++i) {
                    func(i);
                }
            });
        }

        waitAll();
    }
};

} // namespace Baryon::Core
