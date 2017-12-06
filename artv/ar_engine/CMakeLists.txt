file(GLOB tmp *.cpp)
file(GLOB tmp vobjects/*.cpp)
set(CORE_SRCS ${CORE_SRCS} ${tmp})

# ---[ Send the src list to the parent scope.
set(CORE_SRCS ${CORE_SRCS} PARENT_SCOPE)