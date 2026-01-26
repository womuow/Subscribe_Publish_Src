cmake_minimum_required(VERSION 3.10)
project(AES_CBC_Demo VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 设置编译选项
if(MSVC)
    add_compile_options(/W4 /WX)
else()
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
    add_compile_options(-O2)  # 优化级别
endif()

# 查找OpenSSL库
find_package(OpenSSL REQUIRED)
if(NOT OpenSSL_FOUND)
    message(FATAL_ERROR "OpenSSL not found! Please install OpenSSL development package")
endif()

message(STATUS "Found OpenSSL: ${OPENSSL_INCLUDE_DIR} ${OPENSSL_LIBRARIES}")
message(STATUS "OpenSSL Version: ${OPENSSL_VERSION}")


# # 根据OpenSSL版本设置不同的编译选项
# if(OPENSSL_VERSION_MAJOR GREATER_EQUAL 3)
#     # OpenSSL 3.x: 禁用废弃警告
#     if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
#         add_compile_options(-Wno-deprecated-declarations)
#     endif()
# endif()


# 包含目录
include_directories(${OPENSSL_INCLUDE_DIR})

# 添加可执行文件
add_executable(aes_cbc_demo
    DecryptAES256CBC.cpp
)

# 链接OpenSSL库
target_link_libraries(aes_cbc_demo ${OPENSSL_LIBRARIES})

# 设置输出目录
set_target_properties(aes_cbc_demo PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    DEBUG_POSTFIX "_d"
    RELEASE_POSTFIX ""
    RELWITHDEBINFO_POSTFIX "_rd"
    MINSIZEREL_POSTFIX "_ms"
)

# 安装目标
install(TARGETS aes_cbc_demo
    RUNTIME DESTINATION bin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

# 安装配置文件
install(FILES README.md DESTINATION .)