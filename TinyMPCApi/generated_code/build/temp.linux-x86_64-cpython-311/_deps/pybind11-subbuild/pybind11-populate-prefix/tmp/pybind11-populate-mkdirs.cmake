# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-src"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-build"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/tmp"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/src/pybind11-populate-stamp"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/src"
  "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/src/pybind11-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/src/pybind11-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/media/lzx/lzx/lzx/car_simulation/TinyMPCApi/generated_code/build/temp.linux-x86_64-cpython-311/_deps/pybind11-subbuild/pybind11-populate-prefix/src/pybind11-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
