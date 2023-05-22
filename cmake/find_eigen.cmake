# redefine functions to make the script mode work
function(add_library)
endfunction()
function(set_target_properties)
endfunction()

# find eigen
find_package(Eigen3 3.3 NO_MODULE QUIET)

# print result
if(${EIGEN3_FOUND})
    message(NOTICE ${EIGEN3_INCLUDE_DIR})
else()
    message(FATAL_ERROR "Eigen not found.")
endif()
