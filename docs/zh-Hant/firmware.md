接線：

將Arduino Nano連接到USBASP編程器
(ArduinoISP未經過測試，無法使用現有的platformio.ini部分，需要自行編寫) 
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

請注意，連接器引腳可能不同

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

編譯和刷寫固件：

!!! info annotate "configuration.h"

    :arrow_forward:</i> 在configuration.h文件中配置固件。
    

!!! warning annotate "編譯錯誤"  
    如果編譯時出現指示固件大小與微控制器內存大小不匹配的錯誤，通常按順序執行以下命令可以解決
    ```
    pio update
    pio pkg update
    ```

### 選擇秤模塊數量

=== "2個模塊"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3個模塊"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4個模塊"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. 刷寫MiniCore內核

需要USBASP編程器

2.1 更改熔斷器設置


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 刷寫EEPROM

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### 刷寫微控制器

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### 自動PID校準 (PID calibrate)

按需執行

分兩個階段進行：
在configuration.h中設置
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = 秤的數量)
```
執行第2.3步驟進行刷寫
刷寫後，自動PID調整將開始，完成後屏幕上將顯示"Прошей часть 2"消息
改為 #define AUTOPID_RUN 0
並執行第2.3步驟

### 冷卻風扇測試刷寫

按需執行

在configuration.h中取消註釋 
```
// #define PWM_TEST
```
執行第2.3步驟
冷卻風扇將以所有可用頻率進行測試，PWM填充比從100-10%，並將工作模式顯示在屏幕上。測試完成後，根據您對噪音級別和風扇工作質量的偏好，在configuration.h中設置 
```
#define PWM_11_FREQUENCY
```
所需的頻率，註釋掉 
```
#define PWM_TEST
```

進行第2.3步驟刷寫微控制器

!!! warning annotate "啟動錯誤"
    如果在刷寫後或使用過程中屏幕上出現錯誤，請參考configuration.h文件

## 教學視頻
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
視頻相關鏈接：

[VS Code](https://code.visualstudio.com/Download)

[驅動程序](https://zadig.akeo.ie/)

[固件](https://github.com/pavluchenkor/iDryerController)


[Telegram群組 :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[貢獻 :material-file-edit:](https://github.com/pavluchenkor/iDryerController)

