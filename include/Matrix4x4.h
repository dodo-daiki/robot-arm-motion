#pragma once
#include <Arduino.h>

class Matrix4x4 {
public:
    Matrix4x4();                 // コンストラクタ → 単位行列で初期化

    void setIdentity();          // 単位行列にセット
    void multiply(const Matrix4x4& other);  // 自分 *= other
    void copyFrom(const Matrix4x4& other);  // コピー
    void print() const;          // デバッグ用出力

    float data[4][4];            // データ本体
};
