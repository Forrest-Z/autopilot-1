### toolchain.cmake ###
# this is required
SET(CMAKE_SYSTEM_NAME Linux)

# specify the cross compiler
SET(CMAKE_C_COMPILER   /root/335XLBV111_2015-05-12/cross_compiler/linux-devkit/sysroots/i686-arago-linux/usr/bin/arm-linux-gnueabihf-gcc)
SET(CMAKE_CXX_COMPILER /root/335XLBV111_2015-05-12/cross_compiler/linux-devkit/sysroots/i686-arago-linux/usr/bin/arm-linux-gnueabihf-g++)

# where is the target environment
SET(CMAKE_FIND_ROOT_PATH  /root/335XLBV111_2015-05-12/cross_compiler/linux-devkit/sysroots/i686-arago-linux/usr)

# search for programs in the build host directories (not necessary)
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
