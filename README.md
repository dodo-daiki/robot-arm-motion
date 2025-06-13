
# RobotArm - Lightweight Kinematics Library for Arduino (PlatformIO Ready)

A lightweight kinematics library for simple robot arms on Arduino-compatible boards.  
Supports **forward kinematics** and **inverse kinematics (IK)** using fixed-size 4x4 matrix math.  
Designed to run on **embedded targets** (AVR, ESP32, STM32, etc.) with **PlatformIO**.

---

Arduino向けのシンプルなロボットアーム用 **順運動学／逆運動学ライブラリ** です。  
4x4固定長行列による **高速な行列計算** を利用。  
**組み込みマイコン（AVR / ESP32 / STM32など）上で PlatformIO 環境で動作**します。  

---

## Features / 特徴

- Fast **4x4 Matrix** operations (`Matrix4x4` class) / 高速な4x4行列演算
- **Forward kinematics** / 順運動学
- **Inverse kinematics (Jacobian Transpose method)** / 逆運動学（ヤコビアン転置法）
- Simple C++ API for use on microcontrollers / マイコンで扱いやすいシンプルなC++ API
- No external dependencies (no Eigen, no dynamic allocation) / 外部依存なし（Eigen不要／動的メモリ不使用）
- Suitable for **2-3 DOF robot arms** on MCUs / 2～3自由度程度のロボットアームに最適

---

## Installation (PlatformIO) / インストール方法（PlatformIO）

### Option 1: Local library / ローカルライブラリとして利用

Clone this repository into your PlatformIO project's `lib/` folder:

```bash
git clone https://github.com/dodo-daiki/robot-arm-motion.git lib/RobotArm
```

### Option 2: Use `lib_deps` in `platformio.ini`

If published to PlatformIO registry:

```ini
lib_deps = https://github.com/dodo-daiki/robot-arm-motion
```

Or use a local path:

```ini
lib_deps = 
    file://./lib/RobotArm
```

---


## Supported Platforms / 対応プラットフォーム

- AVR (Arduino UNO, Nano, Mega)
- ESP32
- ESP8266
- STM32

---

## Limitations / 制限事項

- Designed for simple **2-3 DOF robot arms** / 5～6自由度程度のロボットアーム向け
- Inverse kinematics uses **Jacobian Transpose** (not optimal for all configurations)  
  逆運動学は **ヤコビアン転置法** を使用（全ての構成に対して最適とは限らない）
- No collision checking / 衝突判定なし

---

## License / ライセンス

MIT License

---

## Author / 作者

百々 大貴 (Daiki Dodo)  
[GitHub: https://github.com/dodo-daiki/robot-arm-motion](https://github.com/dodo-daiki/robot-arm-motion)

---

Pull requests and contributions are welcome!  
プルリクエスト／貢献歓迎します！
