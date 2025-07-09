#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 从当前目录开始，向上查找直到找到 CMakeLists.txt 文件，确定项目根目录
PROJECT_DIR=$SCRIPT_DIR
while [ ! -f "$PROJECT_DIR/CMakeLists.txt" ]; do
  PROJECT_DIR=$(dirname "$PROJECT_DIR")
  # 防止无限循环
  if [ "$PROJECT_DIR" = "/" ]; then
    echo -e "\033[31mError: Could not find CMakeLists.txt in any parent directory.\033[0m"
    exit 1
  fi
done

# 从CMakeLists.txt文件中提取项目名称
CMAKE_PROJECT_NAME=$(grep -E "set\(CMAKE_PROJECT_NAME" "$PROJECT_DIR/CMakeLists.txt" | sed -E 's/set\(CMAKE_PROJECT_NAME[[:space:]]+([^)]+)\)/\1/' | tr -d ' ')
echo -e "\033[36mProject root directory: $PROJECT_DIR\033[0m"
echo -e "\033[36mProject name from CMakeLists.txt: $CMAKE_PROJECT_NAME\033[0m"

# 设置构建类型(默认为Debug，可通过参数修改)
BUILD_TYPE="Debug"
if [ "$1" == "debug" ] || [ "$1" == "Debug" ]; then
  BUILD_TYPE="Debug"
  shift # 移除参数
elif [ "$1" == "release" ] || [ "$1" == "Release" ]; then
  BUILD_TYPE="Release"
  shift # 移除参数
fi
echo -e "\033[35mBuild type: $BUILD_TYPE\033[0m"

# 创建并进入构建目录
BUILD_DIR="$PROJECT_DIR/build"
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR" || { echo -e "\033[31mFailed to enter build directory\033[0m"; exit 1; }

# 运行 CMake 配置和编译项目
echo -e "\033[36mConfiguring project with CMake...\033[0m"
if command -v ninja &> /dev/null; then
  # 如果有 Ninja，使用 Ninja 生成器
  cmake .. -GNinja -DCMAKE_BUILD_TYPE=$BUILD_TYPE
  echo -e "\033[35mBuilding project with Ninja...\033[0m"
  ninja -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
else
  # 否则使用默认生成器
  cmake .. -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=$BUILD_TYPE
  echo -e "\033[35mBuilding project with Make...\033[0m"
  make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
fi

# 检查构建是否成功
if [ $? -ne 0 ]; then
  echo -e "\033[31mBuild failed!\033[0m"
  exit 1
fi

# 根据从CMakeLists.txt提取的项目名称生成文件路径
ELF_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.elf"
BIN_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.bin"
HEX_FILE="$BUILD_DIR/${CMAKE_PROJECT_NAME}.hex"

# 检查 ELF 文件是否成功生成
if [ -f "$ELF_FILE" ]; then
  # 将 ELF 文件转换为 BIN 文件和 HEX 文件
  echo -e "\033[36mConverting ELF to BIN and HEX formats...\033[0m"
  arm-none-eabi-objcopy -O binary "$ELF_FILE" "$BIN_FILE"
  arm-none-eabi-objcopy -O ihex "$ELF_FILE" "$HEX_FILE"
  echo -e "\033[32mBuild completed successfully!\033[0m"
  echo "Output files:"
  echo -e "\033[35m  ELF: $ELF_FILE\033[0m"
  echo -e "\033[35m  BIN: $BIN_FILE\033[0m"
  echo -e "\033[35m  HEX: $HEX_FILE\033[0m"
else
  echo -e "\033[31mError: ELF file not found. Compilation might have failed.\033[0m"
  exit 1
fi
