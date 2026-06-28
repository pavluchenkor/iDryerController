配線:

Arduino NanoをUSBASPプログラマに接続します
(ArduinoISPは既存のplatformio.iniセクションではテストされていません。自分で記述する必要があります)
<!-- https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png -->
![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/pins.png)

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/usbasp10.jpg)

コネクタのピン配置は異なる場合があることに注意してください

![img](https://raw.githubusercontent.com/pavluchenkor/iDryerController/master/src_isp/img001.png)

コンパイルとファームウェア書き込み:

!!! info annotate "configuration.h"

    :arrow_forward:</i> configuration.hファイルでファームウェアを設定します。
    

!!! warning annotate "コンパイルエラー"  
    コンパイル時にファームウェアサイズがマイコンメモリサイズと合わないというエラーが発生した場合、通常は次のコマンドを順番に実行することで解決します
    ```
    pio update
    pio pkg update
    ```

### はかりモジュール数の選択

=== "2モジュール"

    ``` 
    #define SCALES_MODULE_NUM 2
    ```

=== "3モジュール"

    ``` 
    #define SCALES_MODULE_NUM 3
    ```
=== "4モジュール"

    ``` 
    #define SCALES_MODULE_NUM 4
    ```

### 1. MiniCoreコアのファームウェア書き込み

USBASPプログラマが必要です

2.1 ヒューズの変更


=== "Atmega328P"

    ``` 
    pio run -e fuses -t fuses
    ```

=== "Atmega328PB"

    ```
    pio run -e fuses -t fuses
    ```

### 2.2 EEPROMの書き込み

=== "Atmega328P"

    ``` 
    pio run -e EEP -t uploadeep
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t uploadeep
    ```


### マイコンコントローラのファームウェア書き込み

=== "Atmega328P"

    ``` 
    pio run -e EEP -t upload
    ```

=== "Atmega328PB"

    ```
    pio run -e EEPPB -t upload
    ```

### オートPID (PID自動調整)

必要に応じて実行します

2段階で実行します:
configuration.hで以下を設定します
```
 #define AUTOPID_RUN 1
 #define SCALES_MODULE_NUM X (X = はかりの数)
```
手順2.3のファームウェア書き込みを実行します
ファームウェア書き込み後、自動PID調整が開始され、完了するとディスプレイに「Прошей часть 2」と表示されます
#define AUTOPID_RUN 0に変更します
手順2.3を実行します

### クーラーテストのファームウェア書き込み

必要に応じて実行します

configuration.hで次の行をコメント解除します
```
// #define PWM_TEST
```
手順2.3を実行します
すべての利用可能な周波数でクーラーテストが開始され、PWMデューティ比100-10%でディスプレイに動作モードが表示されます。テスト完了後、ノイズレベルとクーラーの動作品質に関する希望に基づいて、configuration.hで以下を設定します
```
#define PWM_11_FREQUENCY
```
希望の周波数を選択し、次をコメントアウトします
```
#define PWM_TEST
```

手順2.3でマイコンを書き込みます

!!! warning annotate "起動時エラー"
    ファームウェア書き込み後および動作中にディスプレイにエラーが表示される場合は、configuration.hファイルを確認してください


## 教育用ビデオ
<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/psw30jXlxPQ" frameborder="0" allowfullscreen></iframe>
</div>
ビデオへのリンク:

[VS Code](https://code.visualstudio.com/Download)

[ドライバ](https://zadig.akeo.ie/)

[ファームウェア](https://github.com/pavluchenkor/iDryerController)


[Telegramグループ :fontawesome-solid-paper-plane:](https://t.me/iDryer){ .md-button }

[Contributing :material-file-edit:](https://github.com/pavluchenkor/iDryerController)
