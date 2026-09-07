# FindTinyXML.cmake
# 查找 TinyXML 库和头文件

find_path(TinyXML_INCLUDE_DIR
    NAMES tinyxml.h
    PATHS /usr/include /usr/local/include
)

find_library(TinyXML_LIBRARY
    NAMES tinyxml
    PATHS /usr/lib /usr/local/lib
)

if(TinyXML_INCLUDE_DIR AND TinyXML_LIBRARY)
    set(TinyXML_FOUND TRUE)
    set(TinyXML_INCLUDE_DIRS ${TinyXML_INCLUDE_DIR})
    set(TinyXML_LIBRARIES ${TinyXML_LIBRARY})
endif()

mark_as_advanced(TinyXML_INCLUDE_DIR TinyXML_LIBRARY)