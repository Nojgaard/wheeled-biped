file(GLOB hdrs *.h)
file(GLOB srs *.cpp)

add_library(${project} STATIC ${hdrs} ${srs})