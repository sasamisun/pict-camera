# CMake version requirement
cmake_minimum_required(VERSION 3.16)

# プロジェクト名
set(PROJECT_NAME "pixel_art_camera")

# ESP-IDFのコンポーネントを含める
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# プロジェクト定義
project(${PROJECT_NAME})