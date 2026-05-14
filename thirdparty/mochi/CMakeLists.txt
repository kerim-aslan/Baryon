include(FetchContent)

FetchContent_Declare(
  mochi_fetch
  GIT_REPOSITORY https://github.com/Jixoid/Mochi.git
  GIT_TAG        387337df763ec45a002ff166f0d307db1778fcb6
  GIT_SHALLOW    FALSE
  GIT_PROGRESS   TRUE
  USES_TERMINAL_DOWNLOAD TRUE
)

FetchContent_MakeAvailable(mochi_fetch)
