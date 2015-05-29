# Help CMake find recent Boost MSVC binaries without manual configuration.
if(MSVC)
    if(NOT DEFINED Boost_USE_MULTITHREADED)
        set(Boost_USE_MULTITHREADED ON) # Most common ABI
    endif()
endif()
if(MSVC AND (NOT Boost_INCLUDE_DIR OR NOT Boost_LIBRARY_DIR))

    math(EXPR _vs_ver "${MSVC_VERSION} / 100 - 6")
    if(CMAKE_CXX_SIZEOF_DATA_PTR EQUAL 8)
        set(_libdir "lib64-msvc-${_vs_ver}.0")
    else()
        set(_libdir "lib32-msvc-${_vs_ver}.0")
    endif()

    set(_haslibs)
    if(EXISTS "c:/local")
        file(GLOB _possibilities "c:/local/boost*")
        list(REVERSE _possibilities)
        foreach(DIR ${_possibilities})
            if(EXISTS "${DIR}/${_libdir}")
                list(APPEND _haslibs "${DIR}")
            endif()
        endforeach()
        if(_haslibs)
            list(APPEND CMAKE_PREFIX_PATH ${_haslibs})
            find_package(Boost QUIET)
            if(Boost_FOUND AND NOT Boost_LIBRARY_DIR)
                set(BOOST_ROOT "${Boost_INCLUDE_DIR}" CACHE PATH "")
                set(BOOST_LIBRARYDIR "${Boost_INCLUDE_DIR}/${_libdir}" CACHE PATH "")
            endif()
        endif()
    endif()
endif()