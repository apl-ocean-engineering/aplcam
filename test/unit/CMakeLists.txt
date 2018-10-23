
gtest_begin(aplcam)
    fips_src( . )

    fips_deps(aplcam g3logger)

    include_directories( ${TEST_DATA_DIR} )
gtest_end()
