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

#include <atomic>
#include <functional>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

namespace Baryon::Debug {

enum class LogCategory : uint32_t {
  COLLISION_NARROW = 0,
  COLLISION_BROAD  = 1,
  SOLVER_IMPULSE   = 2,
  SOLVER_POSITION  = 3,
  ISLAND_SLEEP     = 4,
  GENERAL          = 5
};

/**
 * @class DebugManager
 * @brief Thread-safe, atomik bitmask tabanlı ve geri çağrı (callback) destekli hafif hata ayıklama yöneticisi (B6).
 */
class DebugManager {
public:
  using LogCallback = std::function<void(LogCategory, std::string_view)>;

private:
  std::atomic<uint32_t> m_categoryMask{ (1u << static_cast<uint32_t>(LogCategory::GENERAL)) };
  std::atomic<bool> m_globalEnable{ true };
  LogCallback m_customCallback{ nullptr };

  DebugManager() = default;

public:
  static DebugManager &get() {
    static DebugManager instance;
    return instance;
  }

  void setGlobalEnable(bool state) { m_globalEnable.store(state, std::memory_order_relaxed); }
  [[nodiscard]] bool isGlobalEnabled() const { return m_globalEnable.load(std::memory_order_relaxed); }

  void setCategory(LogCategory cat, bool state) {
    uint32_t bit = 1u << static_cast<uint32_t>(cat);
    if (state) {
      m_categoryMask.fetch_or(bit, std::memory_order_relaxed);
    } else {
      m_categoryMask.fetch_and(~bit, std::memory_order_relaxed);
    }
  }

  [[nodiscard]] bool isCategoryEnabled(LogCategory cat) const {
    if (!m_globalEnable.load(std::memory_order_relaxed)) return false;
    return (m_categoryMask.load(std::memory_order_relaxed) & (1u << static_cast<uint32_t>(cat))) != 0;
  }

  void setCallback(LogCallback callback) {
    m_customCallback = std::move(callback);
  }

  template <typename... Args> void log(LogCategory cat, Args&&... args) {
    if (!isCategoryEnabled(cat))
      return;

    std::ostringstream ss;
    (ss << ... << std::forward<Args>(args));
    std::string msg = ss.str();

    if (m_customCallback) {
      m_customCallback(cat, msg);
    }
  }

  static constexpr std::string_view getCategoryPrefix(LogCategory cat) {
    switch (cat) {
    case LogCategory::COLLISION_NARROW: return "[NARROW-PHASE] ";
    case LogCategory::COLLISION_BROAD:  return "[BROAD-PHASE] ";
    case LogCategory::SOLVER_IMPULSE:   return "[SOLVER-IMP] ";
    case LogCategory::SOLVER_POSITION:  return "[SOLVER-POS] ";
    case LogCategory::ISLAND_SLEEP:     return "[ISLAND-SLEEP] ";
    case LogCategory::GENERAL:          return "[INFO] ";
    default:                            return "[UNKNOWN] ";
    }
  }
};

} // namespace Baryon::Debug

#if defined(BARYON_DISABLE_LOGGING) || (defined(NDEBUG) && !defined(BARYON_ENABLE_LOGGING))
  #define BARYON_LOG_NARROW(...)  do {} while (0)
  #define BARYON_LOG_IMPULSE(...) do {} while (0)
  #define BARYON_LOG_POS(...)     do {} while (0)
  #define BARYON_LOG_SLEEP(...)   do {} while (0)
  #define BARYON_LOG_INFO(...)    do {} while (0)
#else
  #define BARYON_LOG_NARROW(...)                                               \
    do {                                                                       \
      if (Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
              Baryon::Debug::LogCategory::COLLISION_NARROW))                   \
        Baryon::Debug::DebugManager::get().log(                                \
            Baryon::Debug::LogCategory::COLLISION_NARROW, __VA_ARGS__);        \
    } while (0)
  #define BARYON_LOG_IMPULSE(...)                                              \
    do {                                                                       \
      if (Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
              Baryon::Debug::LogCategory::SOLVER_IMPULSE))                     \
        Baryon::Debug::DebugManager::get().log(                                \
            Baryon::Debug::LogCategory::SOLVER_IMPULSE, __VA_ARGS__);         \
    } while (0)
  #define BARYON_LOG_POS(...)                                                  \
    do {                                                                       \
      if (Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
              Baryon::Debug::LogCategory::SOLVER_POSITION))                    \
        Baryon::Debug::DebugManager::get().log(                                \
            Baryon::Debug::LogCategory::SOLVER_POSITION, __VA_ARGS__);         \
    } while (0)
  #define BARYON_LOG_SLEEP(...)                                                \
    do {                                                                       \
      if (Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
              Baryon::Debug::LogCategory::ISLAND_SLEEP))                       \
        Baryon::Debug::DebugManager::get().log(                                \
            Baryon::Debug::LogCategory::ISLAND_SLEEP, __VA_ARGS__);            \
    } while (0)
  #define BARYON_LOG_INFO(...)                                                 \
    do {                                                                       \
      if (Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
              Baryon::Debug::LogCategory::GENERAL))                            \
        Baryon::Debug::DebugManager::get().log(                                \
            Baryon::Debug::LogCategory::GENERAL, __VA_ARGS__);                 \
    } while (0)
#endif
