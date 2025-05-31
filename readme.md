```txt
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
)
```
添加链接选项以支持浮点数打印