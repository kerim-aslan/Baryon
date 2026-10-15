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

#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>

namespace Baryon::Debug {

enum class LogCategory {
  COLLISION_NARROW,
  COLLISION_BROAD,
  SOLVER_IMPULSE,
  SOLVER_POSITION,
  ISLAND_SLEEP,
  GENERAL
};

class DebugManager {
private:
  std::unordered_map<LogCategory, bool> m_categoryStates;
  bool m_globalEnable = true;

  DebugManager() {
    // Varsayılan olarak GENERAL hariç tüm detaylı loglar kapalıdır
    m_categoryStates[LogCategory::COLLISION_NARROW] = false;
    m_categoryStates[LogCategory::COLLISION_BROAD] = false;
    m_categoryStates[LogCategory::SOLVER_IMPULSE] = false;
    m_categoryStates[LogCategory::SOLVER_POSITION] = false;
    m_categoryStates[LogCategory::ISLAND_SLEEP] = false;
    m_categoryStates[LogCategory::GENERAL] = true;
  }

public:
  static DebugManager &get() {
    static DebugManager instance;
    return instance;
  }

  void setGlobalEnable(bool state) { m_globalEnable = state; }
  void setCategory(LogCategory cat, bool state) {
    m_categoryStates[cat] = state;
  }

  [[nodiscard]] bool isCategoryEnabled(LogCategory cat) const {
    if (!m_globalEnable)
      return false;
    auto it = m_categoryStates.find(cat);
    return (it != m_categoryStates.end()) ? it->second : false;
  }

  void outputMessage(LogCategory cat, const std::string &message);

  template <typename... Args> void log(LogCategory cat, Args &&...args) {
    if (!isCategoryEnabled(cat))
      return;

    std::ostringstream oss;
    (oss << ... << std::forward<Args>(args));
    outputMessage(cat, oss.str());
  }

  std::string getCategoryPrefix(LogCategory cat) const {
    switch (cat) {
    case LogCategory::COLLISION_NARROW:
      return "[NARROW-PHASE] ";
    case LogCategory::COLLISION_BROAD:
      return "[BROAD-PHASE] ";
    case LogCategory::SOLVER_IMPULSE:
      return "[SOLVER-IMP] ";
    case LogCategory::SOLVER_POSITION:
      return "[SOLVER-POS] ";
    case LogCategory::ISLAND_SLEEP:
      return "[ISLAND-SLEEP] ";
    case LogCategory::GENERAL:
      return "[INFO] ";
    default:
      return "[UNKNOWN] ";
    }
  }
};

} // namespace Baryon::Debug

// Log makroları: Kategori veya global loglama kapalıysa argümanlar KESİNLİKLE hesaplanmaz.
#define BARYON_LOG_NARROW(...)                                                 \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::COLLISION_NARROW)) {                 \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::COLLISION_NARROW, __VA_ARGS__);        \
    }                                                                          \
  } while (0)

#define BARYON_LOG_IMPULSE(...)                                                \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::SOLVER_IMPULSE)) {                   \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::SOLVER_IMPULSE, __VA_ARGS__);          \
    }                                                                          \
  } while (0)

#define BARYON_LOG_POS(...)                                                    \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::SOLVER_POSITION)) {                  \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::SOLVER_POSITION, __VA_ARGS__);         \
    }                                                                          \
  } while (0)

#define BARYON_LOG_SLEEP(...)                                                  \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::ISLAND_SLEEP)) {                     \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::ISLAND_SLEEP, __VA_ARGS__);            \
    }                                                                          \
  } while (0)

#define BARYON_LOG_BROAD(...)                                                  \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::COLLISION_BROAD)) {                  \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::COLLISION_BROAD, __VA_ARGS__);         \
    }                                                                          \
  } while (0)

#define BARYON_LOG_INFO(...)                                                   \
  do {                                                                         \
    if (::Baryon::Debug::DebugManager::get().isCategoryEnabled(                \
            ::Baryon::Debug::LogCategory::GENERAL)) {                          \
      ::Baryon::Debug::DebugManager::get().log(                                \
          ::Baryon::Debug::LogCategory::GENERAL, __VA_ARGS__);                 \
    }                                                                          \
  } while (0)
