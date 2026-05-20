find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 QUIET)

if(pybind11_FOUND)

    message(STATUS "Compiling python bindings")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR})
    pybind11_add_module(pymcdc pymcdc.cpp)
    target_link_libraries(pymcdc PUBLIC mcdc)

    ament_get_python_install_dir(PYTHON_INSTALL_DIR)
    install(TARGETS pymcdc
            LIBRARY DESTINATION ${PYTHON_INSTALL_DIR})

else()

    message(WARNING "pybind11 not found: skipping Python bindings")

endif()





