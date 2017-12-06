file(GLOB tmp *.cpp)
file(GLOB tmp vobjects/*.cpp)
set(ENGINE_SRCS ${ENGINE_SRCS} ${tmp})

# ---[ Send the src list to the parent scope.
set(ENGINE_SRCS ${ENGINE_SRCS} PARENT_SCOPE)
