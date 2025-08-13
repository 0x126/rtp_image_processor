cmake_minimum_required(VERSION 3.10)

# Find GTest
find_package(GTest)

if(GTest_FOUND)
    # Test executable
    add_executable(test_image_processor test_image_processor.cpp)
    
    target_link_libraries(test_image_processor
        imageprocessor
        GTest::GTest
        GTest::Main
        ${GST_LIBRARIES}
    )
    
    target_include_directories(test_image_processor PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${GST_INCLUDE_DIRS}
    )
    
    # Add test
    add_test(NAME ImageProcessorTest COMMAND test_image_processor)
else()
    message(WARNING "GTest not found. Tests will not be built.")
endif()