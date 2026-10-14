include(FetchContent)

set(ZMS_ZENOHC_VERSION 1.5.0)
set(ZMS_ZENOHCXX_VERSION 1.5.0)
set(ZMS_JSON_VERSION 3.11.3)
set(ZMS_JSON_SHA256 d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d)
set(ZMS_ZENOHCXX_SHA256 aad21519b995a33a7e26db3234ae8d8af5ddd6a6fcd206a23c62cac179c419d8)
set(ZMS_INSTALLABLE TRUE)

if(ZMS_VERSIONS_ONLY)
  return()
endif()

find_package(nlohmann_json ${ZMS_JSON_VERSION} QUIET)
if(NOT nlohmann_json_FOUND AND ZMS_FETCH_MISSING_DEPENDENCIES)
  set(ZMS_INSTALLABLE FALSE)
  FetchContent_Declare(nlohmann_json
    URL https://github.com/nlohmann/json/releases/download/v${ZMS_JSON_VERSION}/json.tar.xz
    URL_HASH SHA256=${ZMS_JSON_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(nlohmann_json)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  message(FATAL_ERROR "nlohmann_json ${ZMS_JSON_VERSION} is required")
endif()

find_package(zenohc ${ZMS_ZENOHC_VERSION} QUIET)
if(NOT zenohc_FOUND)
  message(FATAL_ERROR "zenoh-c ${ZMS_ZENOHC_VERSION} is required as an installed package")
endif()
find_package(zenohcxx ${ZMS_ZENOHCXX_VERSION} QUIET)
if(NOT zenohcxx_FOUND AND ZMS_FETCH_MISSING_DEPENDENCIES)
  set(ZMS_INSTALLABLE FALSE)
  set(ZENOHCXX_ENABLE_TESTS OFF CACHE BOOL "" FORCE)
  set(ZENOHCXX_ENABLE_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(zenohcxx
    URL https://github.com/eclipse-zenoh/zenoh-cpp/archive/refs/tags/${ZMS_ZENOHCXX_VERSION}.tar.gz
    URL_HASH SHA256=${ZMS_ZENOHCXX_SHA256}
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(zenohcxx)
endif()
if(NOT TARGET zenohcxx::zenohc)
  message(FATAL_ERROR "zenoh-cpp ${ZMS_ZENOHCXX_VERSION} with zenoh-c backend is required")
endif()

find_package(urdfdom 4.0 REQUIRED)
