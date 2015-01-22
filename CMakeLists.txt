set(the_description "The OSX  QTKitCapture Component")
ut_add_component(QTKitCapture DEPS utcore utdataflow utvision )

if(APPLE AND CMAKE_COMPILER_IS_CLANGCXX)
    set(HAVE_QTKIT YES)
    list(APPEND QTKIT_LIBRARIES "-framework QTKit" "-framework QuartzCore" "-framework AppKit")

	ut_component_include_directories("src/QTKitCapture" ${UBITRACK_CORE_DEPS_INCLUDE_DIR} ${OPENCV_INCLUDE_DIR} ${TBB_INCLUDE_DIR})
	ut_glob_component_sources(SOURCES "src/QTKitCapture/QTKitCapture.mm")
	ut_create_single_component(${QTKIT_LIBRARIES} ${TBB_ALL_LIBRARIES})
	ut_install_utql_patterns()
ENDIF()
