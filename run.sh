#!/bin/bash

# 获取当前脚本所在目录
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# 设置build、flash和clean脚本的路径
BUILD_SCRIPT="$SCRIPT_DIR/Scripts/build.sh"
FLASH_SCRIPT="$SCRIPT_DIR/Scripts/flash.sh"
CLEAN_SCRIPT="$SCRIPT_DIR/Scripts/clean.sh"

# 检查脚本是否存在
if [ ! -f "$BUILD_SCRIPT" ]; then
  echo -e "\033[31mError: Build script not found at $BUILD_SCRIPT\033[0m"
  exit 1
fi

if [ ! -f "$FLASH_SCRIPT" ]; then
  echo -e "\033[31mError: Flash script not found at $FLASH_SCRIPT\033[0m"
  exit 1
fi

if [ ! -f "$CLEAN_SCRIPT" ]; then
  echo -e "\033[31mError: Clean script not found at $CLEAN_SCRIPT\033[0m"
  exit 1
fi

# 确保脚本具有可执行权限
chmod +x "$BUILD_SCRIPT"
chmod +x "$FLASH_SCRIPT"
chmod +x "$CLEAN_SCRIPT"

# 根据参数执行对应功能
if [ "$1" == "build" ]; then
  echo -e "\033[36m===== 执行构建操作 =====\033[0m"
  # 如果有额外参数，传递给build脚本
  shift
  "$BUILD_SCRIPT" "$@"
elif [ "$1" == "flash" ]; then
  echo -e "\033[36m===== 执行烧录操作 =====\033[0m"
  # 如果有额外参数，传递给flash脚本
  shift
  "$FLASH_SCRIPT" "$@"
elif [ "$1" == "clean" ]; then
  echo -e "\033[36m===== 执行清理操作 =====\033[0m"
  "$CLEAN_SCRIPT"
else
  # 如果没有参数或参数不是build、flash或clean，则依次执行构建和烧录
  echo -e "\033[36m===== 执行构建操作 =====\033[0m"
  "$BUILD_SCRIPT" "$@"  # 将第一个参数作为构建类型传递

  # 检查构建是否成功
  if [ $? -ne 0 ]; then
    echo -e "\033[31m构建失败，中止烧录操作。\033[0m"
    exit 1
  fi

  echo -e "\033[36m===== 执行烧录操作 =====\033[0m"
  "$FLASH_SCRIPT"
fi
