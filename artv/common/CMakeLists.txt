file(GLOB tmp *.cpp)
set(COMMON_SRCS ${COMMON_SRCS} ${tmp})

# ---[ Send the src list to the parent scope.
set(COMMON_SRCS ${COMMON_SRCS} PARENT_SCOPE)