file(GLOB_RECURSE tmp *.cpp *.cc *.c *.hpp *.h)
set(ENGINE_SRCS ${ENGINE_SRCS} ${tmp})

add_library(engine SHARED ${ENGINE_SRCS})

target_link_libraries(engine LINK_PUBLIC common)