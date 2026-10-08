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
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>


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
    // By default, turn everything off except GENERAL
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

  template <typename... Args> void log(LogCategory cat, Args... args) {
    if (!m_globalEnable)
      return;
    if (!m_categoryStates[cat])
      return;

    std::cout << getCategoryPrefix(cat);
    (std::cout << ... << args) << "\n";
  }

private:
  std::string getCategoryPrefix(LogCategory cat) {
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

#define BARYON_LOG_NARROW(...)                                                 \
  Baryon::Debug::DebugManager::get().log(                                      \
      Baryon::Debug::LogCategory::COLLISION_NARROW, __VA_ARGS__)
#define BARYON_LOG_IMPULSE(...)                                                \
  Baryon::Debug::DebugManager::get().log(                                      \
      Baryon::Debug::LogCategory::SOLVER_IMPULSE, __VA_ARGS__)
#define BARYON_LOG_POS(...)                                                    \
  Baryon::Debug::DebugManager::get().log(                                      \
      Baryon::Debug::LogCategory::SOLVER_POSITION, __VA_ARGS__)
#define BARYON_LOG_SLEEP(...)                                                  \
  Baryon::Debug::DebugManager::get().log(                                      \
      Baryon::Debug::LogCategory::ISLAND_SLEEP, __VA_ARGS__)
#define BARYON_LOG_INFO(...)                                                   \
  Baryon::Debug::DebugManager::get().log(Baryon::Debug::LogCategory::GENERAL,  \
                                         __VA_ARGS__)
