cmake_minimum_required(VERSION 2.8)
find_package(CUDA REQUIRED)

SET(CUDA_SEPARABLE_COMPILATION ON)

set( CUDA_HEADER_FILES
    source_files/Point.h
    source_files/RandomnessGenerator.h
    source_files/Ray.h
    source_files/RandomWalk.h
    source_files/header.h
    source_files/barrierConstructor.h
)

set( CUDA_SOURCE_FILES
    source_files/RandomnessGenerator.cu
    source_files/Ray.cu
    source_files/Point.cu
    source_files/barrierConstructor.cu
)


cuda_add_executable(
    randomWalk source_files/main.cu
    ${CUDA_SOURCE_FILES}
    ${CUDA_HEADER_FILES}
    )

