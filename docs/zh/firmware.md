连接：

使用 USBASP 编程器连接 Arduino Nano
(尚未使用现有 platformio.ini 部分测试 ArduinoISP，需要自己编写)
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

请注意，连接器引脚排列可能不同

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

编译和烧录：

!!! info annotate "configuration.h"

    :arrow_forward:</i> 在 configuration.h 文件中配置固件。
    

!!! warning annotate "编译错误"  
    如果编译期间出现错误，指示固件大小与微控制器内存大小不匹配，通常可以通过依次运行以下命令来解决
    ```
    pio update
    pio pkg update
    ```

### 选择秤模块数量

=== "2 个模块"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3 个模块"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4 个模块"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. 烧录 MiniCore 内核

需要 USBASP 编程器

2.1 更改熔丝位

=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 烧录 EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### 烧录微控制器

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### 自动 PID（PID 校准）

根据需要执行

分两个阶段进行：
在 configuration.h 中设置
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = 秤的数量)
```
执行第 2.3 步的烧录
烧录后将自动开始 PID 调整，完成后屏幕上会显示"第 2 部分已烧录"
将其改为 #define AUTOPID_RUN 0
并执行第 2.3 步

### 风扇测试烧录

根据需要执行

在 configuration.h 中取消注释
```
// #define PWM_TEST
```
执行第 2.3 步
将在所有可用频率下开始风扇测试，PWM 填充为 100-10%，工作模式显示在屏幕上。测试完成后，根据您对噪音水平和风扇工作质量的偏好，在 configuration.h 中设置
```
#define PWM_11_FREQUENCY
```
所需频率，注释掉
```
#define PWM_TEST
```

烧录微控制器第 2.3 步

!!! warning annotate "启动错误"
    如果烧录后和使用过程中屏幕上出现错误，请参考 configuration.h 文件

## 教学视频
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
视频链接：

[VS Code](https://code.visualstudio.com/Download)

[驱动程序](https://zadig.akeo.ie/)

[固件](https://github.com/pavluchenkor/iDryerController)


[Telegram 群组 :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[贡献 :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
