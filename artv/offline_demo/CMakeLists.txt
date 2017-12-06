file(GLOB tmp *.cpp)
set(OFFLINE_DEMO_SRCS ${OFFLINE_DEMO_SRCS} ${tmp})

# ---[ Send the src list to the parent scope.
set(OFFLINE_DEMO_SRCS ${OFFLINE_DEMO_SRCS} PARENT_SCOPE)