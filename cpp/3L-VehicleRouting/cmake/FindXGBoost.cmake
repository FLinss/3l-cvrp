# FindXGBoost.cmake

set(XGBOOST_ROOT "" CACHE PATH "Root of XGBoost install (with include/ and lib/)")

# Look for headers
find_path(XGBOOST_INCLUDE_DIR
    NAMES xgboost/c_api.h
    HINTS ${XGBOOST_ROOT}/include $ENV{XGBOOST_ROOT}/include
    PATH_SUFFIXES include)

# Look for library
find_library(XGBOOST_LIBRARY
    NAMES xgboost
    HINTS ${XGBOOST_ROOT}/lib $ENV{XGBOOST_ROOT}/lib
    PATH_SUFFIXES lib)

# Print what was found
message(STATUS "=== XGBoost detection ===")
message(STATUS "XGBOOST_ROOT:       ${XGBOOST_ROOT}")
message(STATUS "XGBOOST_INCLUDE_DIR:${XGBOOST_INCLUDE_DIR}")
message(STATUS "XGBOOST_LIBRARY:    ${XGBOOST_LIBRARY}")

# Make it fail early if not found
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
    XGBoost
    REQUIRED_VARS XGBOOST_INCLUDE_DIR XGBOOST_LIBRARY
    FAIL_MESSAGE "Could not find XGBoost. Set -DXGBOOST_ROOT=/path/to/xgboost/install"
)

# Define imported target
if(NOT TARGET XGBoost::xgboost)
    add_library(XGBoost::xgboost UNKNOWN IMPORTED)
    set_target_properties(XGBoost::xgboost PROPERTIES
        IMPORTED_LOCATION "${XGBOOST_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${XGBOOST_INCLUDE_DIR}")
endif()

message(STATUS "XGBoost found, target: XGBoost::xgboost")
