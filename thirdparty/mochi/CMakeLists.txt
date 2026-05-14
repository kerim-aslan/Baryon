include(FetchContent)

FetchContent_Declare(
  mochi_fetch
  GIT_REPOSITORY https://github.com/Jixoid/Mochi.git
  GIT_TAG        505daa7c0d1a03475e88115cd0ff09114846a6a9
  GIT_SHALLOW    FALSE
  GIT_PROGRESS   TRUE
  USES_TERMINAL_DOWNLOAD TRUE
)

FetchContent_MakeAvailable(mochi_fetch)
