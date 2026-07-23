<!-- 言語 / Language: [English](README.md) | **日本語** -->

# WaterBoader（PIC12F1840）— 非接触・静電容量式 水位センサ

**水槽の壁（ガラス/プラスチック）ごしに、水に触れずに「水あり/水なし」を判定する I2C センサ**です。
中身は 8 ピンのマイコン **PIC12F1840**。差動電極で水を検出し、その結果（＋生の測定値＋自身のダイ温度）を
**I2C スレーブ（既定アドレス `0x20`）** としてホスト（Arduino / ESP32、例：AquaController）へ返します。

据付現場で対話式ウィザードにより校正でき、**校正値をセンサ自身の EEPROM に保存**するので、
**ホストが繋がっていなくても単独で正しく判定を続けます**（冗長化）。

> **これは「2値（水あり/なし）」センサです。**「水位◯％」のような連続量は測りません。相対測定です。

---

## 📚 ドキュメント地図（どれを読めばいい？）

| 目的 | ドキュメント |
|---|---|
| **とにかく動かす（初心者はまずこれ）** | 👉 [docs/ja/install.md](docs/ja/install.md) — 必要なもの・ツール導入・ファーム書込・配線・初回起動 |
| 配線図・電極の作り方・部品表 | [docs/ja/hardware.md](docs/ja/hardware.md) |
| 校正・読み取り・EEPROM自律・故障復帰 | [docs/ja/usage.md](docs/ja/usage.md) |
| ライブラリ API 一覧 | [docs/ja/api.md](docs/ja/api.md) |
| 困ったとき（症状→対処） | [docs/ja/troubleshooting.md](docs/ja/troubleshooting.md) |
| 通信プロトコル/レジスタの厳密仕様 | [docs/SPEC.md](docs/SPEC.md)（英語） |

---

## 何ができて、何ができないか

**できること**
- 非接触で水の有無を判定（ガラス/プラ壁ごし、電極は水に触れない）
- ヒステリシス付きで出力がバタつかない
- 校正値を EEPROM 保存 → ホスト不在でも単独動作
- 生の差動測定値・ダイ温度も取得可
- I2C の一時エラーからの自己復帰（ウォッチドッグ／リセット要因の可視化）

**できないこと（正直な限界）**
- 連続的な水位（％や cm）は出しません。あくまで **water/dry の2値**
- 絶対容量（pF）は返しません（相対値）
- ダイ温度は 1 点校正で **±1℃程度**（過熱警告用途向け、計測器グレードではない）
- ガラス越しの結合は弱め。電極設計で SNR は改善可

---

## 仕組み（なぜ安定するか）

- **差動センシング** … `測定値 = センス電極 − リファレンス電極`。温度・湿度・電源電圧・多くの EMI は
  両電極に等しく乗るので相殺され、**センス電極付近の実際の水だけ**が値を動かします。だから
  **固定しきい値を保存して使い回せる**ほど安定します。
- **水が近づくと値が下がる** … 乾き時と水時の間にしきい値を置き、PIC 側でヒステリシス判定。
- **堅牢性** … ハードウェア I2C(MSSP) を割込みで処理し、測定と読み取りが衝突しない。
  ウォッチドッグ（暴走時自動リセット）／ブラウンアウトリセット／スタックオーバーフローリセット、
  リセット要因レジスタを内蔵。

---

## クイックスタート（全体像だけ先に）

このシステムは **2 枚の基板** が I2C で繋がった構成です。

```
 [PIC12F1840 センサ基板]  ←─ I2C(SDA/SCL) + 電源 ─→  [ホスト: Arduino/ESP32]
   ・差動電極を水槽壁に貼る                            ・WaterBoader ライブラリで読む
   ・ファーム i2c_wb_5V or 3V3 を書込                   ・wb.water() が 1=水 / 0=乾
```

### ① センサ（PIC）にファームを書き込む
`firmware/i2c_wb_5V/i2c_wb_5V.X`（5V用）または `firmware/i2c_wb_3V3/i2c_wb_3V3.X`（3.3V用）を
MPLAB X で開いてビルド→PICkit で書込。詳細 👉 [install.md](docs/ja/install.md)

### ② ホスト（Arduino/ESP32）にライブラリを入れて読む
```cpp
#include <Wire.h>
#include <WaterBoader.h>

WaterBoader wb(0x20);

void setup() {
  Wire.begin();
  wb.begin();              // クラシックAVRなら begin(true) で A4/A5 内部プルアップも有効化
}

void loop() {
  int w = wb.water();      // 1=水あり, 0=水なし, -1=I2Cエラー
  if (w == 1) { /* 警報やポンプ制御など */ }
  delay(500);
}
```

### ③ 据付時に一度だけ校正（対話式ウィザード）
```cpp
WbConfig cfg;
if (wb.calibrate(cfg)) {   // 空気 → 乾いたガラス → 水、各50サンプル
  wb.pushConfig(cfg);
  wb.save();               // PIC の EEPROM に保存 → 以後は単独で判定
}
```
次回起動時は `wb.configValid() == 1` になり、PIC 自身が校正値を復元します。

完全なサンプルは [`examples/WaterBoaderDemo`](examples/WaterBoaderDemo/WaterBoaderDemo.ino)。

---

## リポジトリ構成

```
WaterBoader/
├─ README.ja.md / README.md      日本語 / 英語トップ
├─ src/                          ホスト側 Arduino/ESP32 ライブラリ (WaterBoader.h/.cpp)
├─ examples/WaterBoaderDemo/     全API を使うサンプルスケッチ
├─ firmware/                     PIC ファーム（MPLAB X プロジェクト）
│   ├─ i2c_wb_5V/i2c_wb_5V.X/      5V 用
│   ├─ i2c_wb_3V3/i2c_wb_3V3.X/    3.3V 用
│   └─ gen_3v3.sh                  5V ソースから 3.3V 版を再生成
├─ hardware/                     KiCad 回路図
├─ docs/                         ドキュメント（ja/ に日本語、SPEC.md に厳密仕様）
├─ library.properties / library.json   Arduino / PlatformIO メタデータ
└─ LICENSE                       MIT
```

---

## ライセンス
MIT License（[LICENSE](LICENSE)）。自由に利用できます。
