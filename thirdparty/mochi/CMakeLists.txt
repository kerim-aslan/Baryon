include(FetchContent)

FetchContent_Declare(
  mochi_fetch
  GIT_REPOSITORY https://github.com/Jixoid/Mochi.git
  GIT_TAG        d0adc3f791c528ca9385f3391d46c0f0a94f6345
  GIT_SHALLOW    FALSE
  GIT_PROGRESS   TRUE
  USES_TERMINAL_DOWNLOAD TRUE
)

FetchContent_MakeAvailable(mochi_fetch)
