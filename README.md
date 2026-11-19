# FlyLens 85 · 실내 드론 1차

FlyLens 85 V1.3 O3 Lite/O4 · 2S 푸셔 · ArduCopter 4.7.2 · 전면 MJPEG

**제작 초안.** 프레임 장착과 저전압 추력 실측이 남아 있다.

| 파일 | 용도 |
| --- | --- |
| [bom.csv](bom.csv) · [wiring.csv](wiring.csv) | 부품, 수량, 참고 단가(2026-11-19), 핀과 모터 방향 |
| [assembly.json](assembly.json) | 배치 원본. mm, +X 우측, +Y 전방, +Z 위 |
| [hardware.parm](hardware.parm) · [sitl.parm](sitl.parm) | 실기 초기 설정 · 기본 Quad‑X 시뮬레이션 설정 |

## 조립과 설정

1. FC 화살표가 전방을 향하게 45°로 장착한다. Ø3 mm의 25.5×25.5 mm 홀 패턴, 약 51 mm의 회전 외곽과 USB 접근 공간을 가조립으로 확인한다. 전면 카메라는 전방, MTF‑02P는 바닥을 향한다. 모터 회전은 기체 위에서 본 방향이다. [프레임](https://flywoo.net/products/flylens-85-frame-kit) · [FC](https://micoair.com/en/docs/flight-controller/micoair743-aio-series/micoair743v2-aio-35a-manual)
2. 배선표대로 연결한다. MTF‑02P를 `mav_apm`으로 설정한다. MG‑903은 I²C 나침반을 사용한다. 전체 5V 부하가 FC 정격 2 A 이내인지 확인한다. [MTF‑02P](https://micoair.com/en/docs/sensors/sensors/mtf-02-02p-sensors) · [MG‑903](https://micoair.com/en/docs/gps-rtk/gps)
3. ArduCopter 4.7.2 MicoAir743v2와 `hardware.parm`을 적용하고 재부팅한다. 프로펠러를 뺀 상태에서 IMU·나침반·RC를 보정하고 2S 배터리 전압 표시를 확인한다. 모드 스위치에 AltHold·Loiter·Land를 배정하고 모터 번호·방향과 RC/PC 링크 단절 동작을 확인한다. Loiter와 Guided에 EKF 원점과 유효한 위치 추정이 필요하다. 광류 최소 거리는 80 mm다. [펌웨어](https://firmware.ardupilot.org/Copter/stable-4.7.2/MicoAir743v2/arducopter.apj) · [광류와 EKF 원점 설정](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
4. WiFi-Link는 5 GHz STA로 연결하고 PC IP의 UDP `14550`을 지정한다. XIAO에는 Arduino ESP32 3.3.7의 CameraWebServer 예제를 올린다. `CAMERA_MODEL_XIAO_ESP32S3`와 PSRAM을 활성화하고 같은 LAN의 2.4 GHz Wi-Fi를 설정한다. 영상은 `http://<camera-ip>:81/stream`에서 읽는다. [WiFi-Link](https://micoair.com/en/docs/telemetry/micoair-wifi-link/micoair-wifi-link-wifi6-telemetry) · [카메라 예제](https://github.com/espressif/arduino-esp32/tree/3.3.7/libraries/ESP32/examples/Camera/CameraWebServer) · [XIAO 설정](https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/)

PC 제어는 Guided에서 `SET_POSITION_TARGET_LOCAL_NED`, 프레임 `LOCAL_NED`, 마스크 `1479`를 주기적으로 보낸다. `vx/vy`는 방 좌표를 NED로 변환한 m/s, `vz`는 하강이 양수인 m/s, `yaw_rate`는 rad/s다. 중립은 네 값이 0이다. 이후 손 기울기·높이 레버·회전 다이얼도 같은 입력을 사용한다. [MAVLink 명령](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html)

2S 저전압 기준은 순정 설정의 셀당 3.5/3.3 V를 적용한 7.0/6.6 V다. 저전압 시 Land로 전환한다. [Flywoo 설정](https://cdn-files.myshopline.com/file/store/1673593876355/s85-o4pro-bf451-flywoof405s-aio-elrs.pdf) · [ArduPilot 배터리 페일세이프](https://ardupilot.org/copter/docs/failsafe-battery.html)

## 물리 검사

```sh
python -m pip install -r requirements.txt
python validate_physics.py --output validation_report.json
python build_stage.py
```

MuJoCo는 배치의 간섭과 가정 추력의 이륙을 검사한다. `flylens85.usda`는 Isaac Sim에서 연다. 배치의 가드·받침대는 추정 치수다. 이륙 중량 추정은 107.9 g이다. 실제 장착과 호버 여유는 가조립 치수와 가드 포함 2S 저전압 총추력으로 확인한다. SITL은 4.7.2의 Quad 모델에 `sitl.parm`을 적용한다.
