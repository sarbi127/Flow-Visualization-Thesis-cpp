#--------------------------------------------------------------------
# Inviwo Example Module
ivw_module(Flow)

#--------------------------------------------------------------------
# Add header files
set(HEADER_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/flowmodule.h
	${CMAKE_CURRENT_SOURCE_DIR}/flowmoduledefine.h
	
)
ivw_group("Header Files" ${HEADER_FILES})

#--------------------------------------------------------------------
# Add source files
set(SOURCE_FILES
	${CMAKE_CURRENT_SOURCE_DIR}/flowmodule.cpp

    	
)
ivw_group("Source Files" ${SOURCE_FILES})

#--------------------------------------------------------------------
# Create module
ivw_create_module(${SOURCE_FILES} ${HEADER_FILES})
