# API リファレンス（ホスト側 Arduino/ESP32 ライブラリ）

ヘッダ：[`src/WaterBoader.h`](../../src/WaterBoader.h) ／ 厳密な通信仕様：[docs/SPEC.md](../SPEC.md)

```cpp
#include <Wire.h>
#include <WaterBoader.h>
WaterBoader wb(0x20);   // 既定アドレス 0x20
```

> **エラー表現**：ライブ測定系は I2C 失敗時に **`-1`（整数）／`NAN`（float）** を返します。
> 設定系（`bool` 返し）は失敗時 `false`。各操作は内部で `WB_RETRY`(3) 回再試行します。

---

## 初期化

| メソッド | 説明 |
|---|---|
| `WaterBoader(uint8_t addr = 0x20)` | コンストラクタ。I2C アドレス指定 |
| `void begin(bool avrPullups = false)` | 状態初期化。**先に `Wire.begin()` を呼ぶこと**。クラシックAVRで `true` を渡すと A4/A5 の内部プルアップも有効化（ESP32 では無効＝no-op） |

---

## ライブ測定（読み取り）

| メソッド | 返り値 | 説明 |
|---|---|---|
| `int water()` | 1=水 / 0=乾 / -1=err | **PIC のヒステリシス判定結果**（最終出力） |
| `long value()` | 差動値 / -1 | 窓トリム済み差動値（0x11）。20000中心・水で下がる |
| `long sense()` | 生値 / -1 | センス電極の生カウント（0x22） |
| `long ref()` | 生値 / -1 | リファレンス電極の生カウント（0x23） |
| `long tempRaw()` | 0..1023 / -1 | ダイ温度の生10bit ADC（0x07） |
| `float tempC()` | ℃ / NAN | 温度（AN1333 Eq.5＋1点校正）。要 `setTempCal` |
| `int tempReady()` | 1/0/-1 | 温度レディ（0x05） |
| `int waterReady()` | 1/0/-1 | 水位値レディ（0x06） |

---

## 設定（PIC がこの値で水を判定）

| メソッド | 説明 |
|---|---|
| `bool setBoundary(uint16_t v)` | ON しきい値 `wbound`（0x03） |
| `bool setHysteresis(uint16_t v)` | ヒステリシス幅 `hyst`（0x04）。OFF = wbound + hyst |
| `bool setGate(uint8_t g)` | ゲート長（0x20、約100µs単位、1..200）。50Hz=200 / 60Hz≈167 |
| `bool setOutSel(uint8_t s)` | 出力選択（0x21）：0=差動(運用) 1=センス 2=リファレンス（デバッグ用） |
| `bool setCal(uint16_t air, uint16_t dry, uint16_t water)` | 校正参照値（0x24-26） |
| `bool getConfig(WbConfig& c)` | 現在の全設定を読み出す |
| `bool pushConfig(const WbConfig& c)` | 全設定をまとめて書き込む |

`WbConfig` 構造体：
```cpp
struct WbConfig {
  uint8_t  gate;
  uint16_t wbound;
  uint16_t hyst;
  uint16_t calAir;
  uint16_t calDry;
  uint16_t calWater;
};
```

---

## EEPROM 永続化 / 自律

| メソッド | 説明 |
|---|---|
| `bool save()` | 設定を PIC の EEPROM に保存（0x0A に 0xA5 書込）。以後ホスト不在でも自律 |
| `int configValid()` | 起動時に EEPROM から有効設定を読めたか（0x0B）：1=有効 |

---

## 健全性 / 復帰

| メソッド | 説明 |
|---|---|
| `int resetCause()` | 最後のリセット要因ビット（0x0C、-1=err） |
| `void printResetCause(uint8_t rc, Stream& io = Serial)` | 要因を人間可読で表示 |
| `bool reset()` | ソフトリセット（0x00） |
| `bool recover(Stream& io = Serial)` | リセット→WDT復帰待ち→要因表示→設定再確認 |

リセット要因ビット（`WB_RST_*`）：
`POR(0x01) / BOR(0x02, 電圧サグ) / WDT(0x04, 暴走自動復帰) / MCLR(0x08) / soft-RESET(0x10)`

---

## 校正ウィザード

```cpp
bool calibrate(WbConfig& out, Stream& io = Serial);
```
`io` に対話プロンプトを出し、**空気 / 乾いたガラス / 水** を各50サンプル取得。
`wbound = 中点(乾, 水)`、`hyst = 0.7 × σ_乾` を計算して `out` に格納します。
**push/save はしません**（呼び出し側が明示的に `pushConfig()`＋`save()` する設計）。
乾と水が分離しない（`乾 ≤ 水`）場合は `false`。

---

## 温度校正

```cpp
void setTempCal(float vdd, float mode, int offset);
```
- **5V 既定**：`setTempCal(4.8, 4.0, 94)`（VDD=4.8, mode=4=高レンジ, offset=94）
- **3.3V**：ファームを `TEMP_HIGH_RANGE=0`（低レンジ）で焼き、`setTempCal(3.3, 2.0, N)`。
  `N` の再導出：
  ```
  Vt      = 0.659 − (T + 40) × 0.00132        // T=既知温度[℃]
  idealADC = (3.3 − 2×Vt) / 3.3 × 1023
  N        = idealADC − rawADC                 // rawADC は tempRaw() の実測
  ```
- 1 点校正なので校正点付近が正確。広域精度が要るなら 2 点校正が必要。

---

## 低レベルアクセス（上級）

| メソッド | 説明 |
|---|---|
| `int  read8(uint8_t reg)` | 1バイトレジスタ読み |
| `long read16(uint8_t reg)` | 2バイトレジスタ読み（下位バイト先） |
| `bool write8(uint8_t reg, uint8_t v)` | 1バイト書き |
| `bool write16(uint8_t reg, uint16_t v)` | 2バイト書き |

レジスタ番号は `WbReg` 列挙（`WB_R_*`）と [SPEC.md](../SPEC.md) の表を参照。

---

## レジスタマップ早見表

| addr | 名称 | R/W | 長さ | 意味 |
|---|---|---|---|---|
| 0x00 | ResetDevice | W | 1 | ポインタ書込でソフトリセット |
| 0x03 | WaterBoarderValue | R/W | 2 | ON しきい値 |
| 0x04 | WaterHisWidth | R/W | 2 | ヒステリシス幅 |
| 0x05 | ReadyTemp | R | 1 | 温度レディ |
| 0x06 | ReadyWater | R | 1 | 水位レディ |
| 0x07 | GetTemp | R | 2 | 生10bit温度 |
| 0x08 | GetWater | R | 1 | 1=水 / 0=乾 |
| 0x0A | SaveConfig | W | 1 | 0xA5 で EEPROM 保存 |
| 0x0B | ConfigValid | R | 1 | EEPROM設定有効 |
| 0x0C | ResetCause | R | 1 | リセット要因 |
| 0x10 | ReadyNCO | R | 1 | 値レディ |
| 0x11 | GetNCORaw | R | 2 | 窓トリム済み差動値（既定read） |
| 0x20 | Gate | R/W | 1 | ゲート長 |
| 0x21 | OutSel | R/W | 1 | 0=差動 1=センス 2=リファレンス |
| 0x22 | SenseRaw | R | 2 | センス生値 |
| 0x23 | RefRaw | R | 2 | リファレンス生値 |
| 0x24 | CalAir | R/W | 2 | 校正参照：空気 |
| 0x25 | CalDry | R/W | 2 | 校正参照：乾ガラス |
| 0x26 | CalWater | R/W | 2 | 校正参照：水 |
