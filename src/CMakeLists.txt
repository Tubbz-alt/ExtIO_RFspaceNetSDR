cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(extio)


SET(CLSOCKET_HEADERS
  ../../clsocket/src/ActiveSocket.h
  ../../clsocket/src/Host.h
  ../../clsocket/src/PassiveSocket.h
  ../../clsocket/src/SimpleSocket.h
  ../../clsocket/src/StatTimer.h
)

SET(CLSOCKET_SOURCES
  ../../clsocket/src/SimpleSocket.cpp
  ../../clsocket/src/PassiveSocket.cpp
)


SET(TTPP_HEADERS
  ../../tinythreadpp/source/tinythread.h
  ../../tinythreadpp/source/fast_mutex.h
)

SET(TTPP_SOURCES
  ../../tinythreadpp/source/tinythread.cpp
)


SET(EXTIO_HEADERS
  ExtIO_Logging.h
  ExtIO_RFspaceNetSDR.h
  LC_ExtIO_Types.h
  procitec_replacements.h
  rfspace_netsdr_control.h
  rfspace_netsdr_receiver.h
  rfspace_netsdr_udpdata.h
)

SET(EXTIO_SOURCES
  ExtIO_Logging.cpp
  ExtIO_RFspaceNetSDR.cpp
  procitec_replacements.cpp
  rfspace_netsdr_control.cpp
  rfspace_netsdr_receiver.cpp
  rfspace_netsdr_udpdata.cpp
)

SET(NETSDREMU_HEADERS
  LC_ExtIO_Types.h
  ExtIO_Logging.h
  rfspace_netsdr_control.h
)

SET(NETSDREMU_SOURCES
  NetSDREmu.cpp
  ExtIO_Logging.cpp
  rfspace_netsdr_control.cpp
)


INCLUDE_DIRECTORIES( ../../clsocket/src ../../tinythreadpp/source )

# mark headers as headers...
SET_SOURCE_FILES_PROPERTIES( ${CLSOCKET_HEADERS} PROPERTIES HEADER_FILE_ONLY TRUE )
SET_SOURCE_FILES_PROPERTIES( ${TTPP_HEADERS} PROPERTIES HEADER_FILE_ONLY TRUE )
SET_SOURCE_FILES_PROPERTIES( ${EXTIO_HEADERS} PROPERTIES HEADER_FILE_ONLY TRUE )
SET_SOURCE_FILES_PROPERTIES( ${NETSDREMU_HEADERS} PROPERTIES HEADER_FILE_ONLY TRUE )

# append to sources so that dependency checks work on headers
LIST(APPEND CLSOCKET_SOURCES ${CLSOCKET_HEADERS})
LIST(APPEND TTPP_SOURCES ${TTPP_HEADERS})
LIST(APPEND EXTIO_SOURCES ${EXTIO_HEADERS})
LIST(APPEND NETSDREMU_SOURCES ${NETSDREMU_HEADERS})

add_definitions(-DEXTIO_EXPORTS)


# OS and compiler checks. 
if(UNIX)
    # linux / normal unix
    add_definitions(-D_LINUX)
    if(CYGWIN)
        # Special Cygwin stuff here
    elseif(APPLE)
        # Special Apple stuff here
        remove_definitions(-D_LINUX)
        add_definitions(-D_DARWIN)
    endif()
    # assume GCC on Unix/Linux - here shared linking is no real issue
    #set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static -s")
    #set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -static -s")
elseif(WIN32)
    add_definitions(-DWIN32)
    SET(PROJECT_LIBS Ws2_32.lib)
    if(MINGW)
        # Special MINGW stuff here
        OPTION(CLSOCKET_OWN_INET_PTON "Use own inet_pton() implementation (required on MINGW)" ON)
        if(CLSOCKET_OWN_INET_PTON)
            add_definitions(-DCLSOCKET_OWN_INET_PTON)
        endif()
        set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_C_FLAGS} -static -static-libgcc -s")
        set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS} -static -static-libgcc -static-libstdc++ -s")
    elseif(MSVC)
        # Special MSVC stuff here
        add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
        set(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -MT")
        set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -MT")
        set(CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -MT")
        set(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -MT")
        set(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -MT")
        set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -MT")
        set(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -MTd")
        set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -MTd")
    else()
        # No idea what it is, but there's a good chance it's too weird.
        MESSAGE( FATAL_ERROR "Using unknown WIN32 compiler... NOT. Please add to build system." )
    endif()
endif()


ADD_LIBRARY( ExtIO_RFspaceNetSDR SHARED ${EXTIO_SOURCES} ${CLSOCKET_SOURCES} ${TTPP_SOURCES} )
TARGET_LINK_LIBRARIES( ExtIO_RFspaceNetSDR ${PROJECT_LIBS} )

ADD_EXECUTABLE( netsdremu ${NETSDREMU_SOURCES} ${CLSOCKET_SOURCES} )
TARGET_LINK_LIBRARIES( netsdremu ${PROJECT_LIBS} )

set(CXX_STANDARD_REQUIRED ON)
set_property(TARGET netsdremu PROPERTY CXX_STANDARD 11)
set_property(TARGET ExtIO_RFspaceNetSDR PROPERTY CXX_STANDARD 11)

set_property(TARGET netsdremu PROPERTY CXX_EXTENSIONS ON)
set_property(TARGET ExtIO_RFspaceNetSDR PROPERTY CXX_EXTENSIONS ON)

