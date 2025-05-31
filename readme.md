# 程序主入口
`Core\Src\main.c`

# 启动烧录
# 全部执行（先构建再烧录）
```bash
./run.sh

# 全部执行（Debug模式）
./run.sh debug

# 仅构建
./run.sh build

# 仅构建（Debug模式）
./run.sh build debug

# 仅烧录
./run.sh flash

# 仅烧录指定文件
./run.sh flash path/to/firmware.hex

# 清理构建文件
./run.sh clean
```


# 引脚图
以下是STM32F103C8T6引脚图
![](./C8T6引脚图.png)

# 增加Utils部分

在`Utils/`下创建控制模块

在根目录下的CMakeLists.txt下修改如下
```diff
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
+    Utils
)
```

# 增加浮点数打印
在根目录下`CMakeLists.txt` 增加

```diff
+ target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
+     -u _printf_float
+ ) 
```
添加链接选项以支持浮点数打印