# MQTT_Dobot_Nova2_Control
Control DOBOT Nova2 Through MQTT　by N.Kawaguchi (2024/06/03)

MQTT でのメッセージを受け取って、 DOBOT Nova2 を直接制御(dobot_api 経由）
(Nova2 のバージョン：3.5.7.0-stable )

Dobot API は、
　　 https://github.com/Dobot-Arm/TCP-IP-CR-Python-V4.git
のファイルを利用


--
課題 (2024/6/3 時点)

- ERROR からの自動復帰
  -> なぜか Jog をしてからでないと、ServoP が動かない
    (自動復帰の仕組みつくるべき？)

- Inverse Kinematics のエラー多発

