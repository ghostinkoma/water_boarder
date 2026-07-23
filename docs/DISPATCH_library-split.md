# WaterBoader — ライブラリ独立化 引き継ぎDISPATCH（2026-07-22）

> このファイル1枚で冷スタート再開可。**新スレッドの主題＝WaterBoader を「第三者が使えるスタンドアロン Arduino/PlatformIO ライブラリ」として独立リポジトリに切り出し、README/ドキュメントを整備する**こと。中核実装・実機検証は完了済み。残タスクはパッケージング＋ドキュメントが主。
>
> **なぜ独立させるか**: 「非接触の water/dry 判定」という一見ありふれた機能に、実際は**約3週間の工数**を要した（CPS成立条件・EMI対策・オーバーフローbug・EEPROM自律化・堅牢化）。資産として再利用可能な形（ライブラリ＋良質な README）に固めることが最重要。

---

## 0. 現在地（完成済み・実機実証）

- **PIC ファーム** `i2c_wb_5V.c` / `i2c_wb_3V3.c`（`D:\hobby\water_boarder\PIC12F18XX\`）: MSSPハードI2Cスレーブ0x20＋差動CPS非接触水位＋ダイ温度10bit＋EEPROM自律化＋WDT/BOR堅牢化。XC8ビルド確認済（各1538w/37.5%, RAM 164B/64%）。**5V/3.3Vでファーム分離**(差は`TEMP_HIGH_RANGE`のみ, 5Vを正として`gen_3v3.sh`で3V3生成)。**3.3V検証=完了**: 水位PASS(再校正gap54〜81), 温度PASS(低レンジ raw669@30°C→**`setTempCal(3.3,2.0,3)`**)。ConfigValid保存レースも修正済(config_save先頭でvalid=0+ホストポーリング)。
- **Arduino/ESP32 ライブラリ** `WaterBoader/`（`D:\hobby\water_boarder\PIC12F18XX\WaterBoader\`）: クラス `WaterBoader`。`WaterBoader.h/.cpp`, `examples/WaterBoaderDemo/`, `library.properties`, `README.md`, `SPEC.md`。
- **参照統合（実証）**: AquaController（ESP32-C3, `D:\hobby\AquaController`）へ結合し **PlatformIO ビルド成功**（Flash 76.3%）。`lib/WaterBoader/` に配置、`sensors.cpp`の`readWaterLevel()`＋`config.h`の`waterlevel`名前空間＋`state.h`の`Live`フィールド。→ 実運用のリファレンスとして使える。
- 既存の詳細記録: 親DISPATCH `D:\hobby\water_boarder\PIC12F18XX\DISPATCH.md`（CPS/温度/EEPROM/WDTの全経緯）。メモリ索引 = `MEMORY.md`。

## 1. 新スレッドの成果物（TODO）

- [ ] **独立リポジトリ構成**（例: `waterboader/` 単独 repo, `git init`）。標準ライブラリ構成:
      `src/WaterBoader.{h,cpp}` / `examples/WaterBoaderDemo/` / `library.properties` / `LICENSE`(MIT等) / `firmware/i2c_wb_5V.c`+`i2c_wb_3V3.c`+`gen_3v3.sh`（PIC側も同梱）/ `docs/`。
- [ ] **README 充実**（下記§2の章立て）。ひとまずこれが最重要。
- [ ] **推定容量分解能**の記載（§3）＋（任意）**LCRメータで電極絶対容量を実測**して確定。
- [ ] **EMI対策とテスト結果**の記載（§4）。
- [ ] **エラー回避/堅牢化**の記載（§5）。
- [ ] Arduino Library Manager / PlatformIO Registry 登録可能な形へ（`library.json`追加, keywords, semver）。
- [ ] （任意）CI: PIC=XC8ビルド、Arduino=arduino-cli/pio でExampleコンパイルチェック。

## 2. README に起こすべき章（第三者視点で「使える」ことが最優先）

1. **概要**: 非接触・静電容量式 water/dry 検出の I2C スレーブ。何ができて何ができないか（水位%ではなく2値。相対測定）。
2. **配線**: ピン表（sense=RA4/pin3, ref=RA0/pin7, SDA=RA2, SCL=RA1, 4.7kプルアップ, 同一電源電圧）。**電極の作り方**（§6の実測条件を明記）。
3. **クイックスタート**: `#include <WaterBoader.h>` → `wb.begin()` → `wb.water()`。最小サンプル。
4. **Example の走らせ方**: `examples/WaterBoaderDemo` を Arduino IDE / `pio run -t upload` で焼く手順、シリアル115200、C/S プロンプト、期待出力。
5. **キャリブレーション手順**: 3状態ウィザード（air→dry glass→water 各50サンプル）、`calibrate()→pushConfig()→save()`、EEPROM自律化（据付後ホスト不在でも動く）。
6. **API リファレンス**: 全メソッド（測定/設定/EEPROM/健全性/校正）。`SPEC.md` のレジスタ表へリンク。
7. **推定分解能・SNR**（§3）。
8. **EMI対策**（§4）と**エラー回避**（§5）。
9. **既知の限界**: 弱結合ガラス越しは gap が小さい／温度は±1°C／絶対容量は非提供。
10. **PIC ファームの焼き方**: XC8 コマンド、MPLAB直貼り運用、ICSP注意（SCL=ICSPCLK兼用）。

## 3. 推定容量分解能（調査結果 2026-07-22・要バーンチ確認）

CPS 発振器は電極容量 C を時定数に持つ緩和発振で、TMR0 が gate 時間中の発振周期を計数。**計数 N ∝ 1/C** ゆえ `ΔC/C = −ΔN/N`。

実測（gate=200≈20ms, 5V）:
- 生カウント `N_sense ≈ 2200`
- water(約20cc,ガラス瓶越し) vs dry の差動信号 `Δ ≈ 88〜103 カウント`（senseがほぼ全変化, refは安定）
- 窓トリム後ノイズ `σ ≈ 2〜6 カウント`（代表4）

導出:
- **分数分解能** `σ/N ≈ 4/2200 ≈ 0.18%`（電極容量の 0.18%/1σ を分離）
- **水信号** `Δ/N ≈ 90/2200 ≈ 4%`（20cc の水がガラス越しに電極容量を約4%変える）
- **SNR ≈ Δ/σ ≈ 90/4 ≈ 22:1**

絶対容量への換算（**要 LCR 実測**）: 2cm/AWG24 電極の自己+結合容量を仮に 2〜10pF とすると:
- 分解能 ≈ 0.18% × (2〜10pF) ≈ **数〜20 fF（1σ）**
- 水信号 ≈ 4% × (2〜10pF) ≈ **0.08〜0.4 pF**

→ オーダーとして「**数十fF分解能・サブpF信号**」。新スレッドで**電極をLCRメータ実測**し絶対pFを確定すれば分解能を数値保証できる（README の説得力に直結）。CPSRNG/gate を変えた感度トレードオフ表も付けると良い。

## 4. EMI対策と検証（実機記録より）

- **差動リファレンス電極（最重要）**: `value = sense(CPS3/RA4) − ref(CPS0/RA0)`。共通モードの RF/商用ハム/温度/湿度/VDD ドリフトを相殺、水信号は sense のみゆえ残る。
  - **効果実証**: 単一電極では **435MHz 5W handheld を60cm以内で偽wet**（持続RFはフィルタで除去不可）。差動化後は **144/435MHz@5W が50cm以上で正常検出**。
  - 逐次測定（sense→ref交互）なので**定常キャリアは良く相殺、パルス性RFは不完全**（同時刻でない）。UHF強干渉時はフェライト＋数十pF＋同軸シールド併用を推奨（README注記）。
- **NPLC積分（gate=200≈20ms=50Hz1周期）**: 開放電極が拾う商用ハムを1周期積分で相殺。**相対ノイズ −9%→−2%（約3倍改善）**、分布の負偏り→ほぼ対称化。60Hz地域は gate=167。
- **中心クラスタ/窓トリム平均（WIN=16）**: 上下4個を捨て中央8平均（整数のみ）。モーター/調光PWM/ヒーター由来の heavy-tailed 外れ値・孤立スパイクを除去。
- **検証規模**: 分離検証 **46万サンプル**、エージング **24189サンプル**ヒストグラムで閾値確定。dry床とglass-waterの間に明瞭ギャップを確認。

## 5. エラー回避 / 堅牢化（ライブラリの売り）

- **★window_estimate uint16オーバーフローbug（教訓・必ずREADMEかコメントに）**: 8個×~19000の合計が uint16 を周回し value が破損（~2500）＋境界でグリッチ。**uint32 で修正**。値空間が変わるため EEPROM magic を bump して旧config自動無効化→再校正誘導。delta方式で長く気付かなかったのは baseline/value 両方が同じく周回して自己整合していたため。
- **割込み駆動 I2C**: 測定20msブロック中でも SSP割込みが各バイトを数µsで応答＝**読み取りが測定と衝突しない**。BME280流にメモリの確定スナップショットを返す（BUSY/READY不要）。
- **ホスト側リトライ** `WB_RETRY=3`（一時MISS吸収）＋ **`recover()`**（reset要求→WDT自動復帰~1s待ち→`resetCause()`で「なぜ止まったか」表示→ConfigValid再確認）。
- **PIC自己修復**: WDTE=ON(周期1s,CLRWDT)で暴走→自動リセット→`config_load()`でEEPROM設定再ロード。BOREN=ON(電圧サグ)、STVREN=ON(スタック)。`ResetCause`(0x0C)で要因可視化。
- **2バイト送信**: master NACK時も CKP を必ず解放（SCLストレッチのバスハング防止）。
- **MISS実務対策**: マスタ側 SDA/SCL 明示プルアップ＋**同一電源電圧**（3.3V/5V混在でMISS多発）＋外付け4.7k。

## 6. 物理セットアップ / テスト条件（実測環境・README必須）

- **対象**: 約 **20cc の水**、**ただのガラス瓶**（水槽壁の代用）。water/dry 2値判定。
- **電極**: **AWG24 単線**、**ガラス面接触長 約2cm**（sense=CPS3/RA4）。リファレンス電極（CPS0/RA0）を背中合わせに配置し差動化。
- **条件**: gate=200(~20ms), 5V, INTOSC 16MHz。
- **キャリブレーション実測（この構成）**: `air=19110 / dry(乾ガラス吸着)=18976 / water(20cc越し)=18888`（gap 88, σ_dry 5）→ `wbound=18932, hyst=0.7σ`。
- 注意: 結合が弱い（gap88）ので、電極接触長を伸ばす/ガラスを薄くする/GNDガードで SNR 改善余地あり。CPSはブレッドボード不可（治具/実装が成立条件）。

## 7. ファイル/参照ポインタ

| 種別 | パス |
|---|---|
| PIC ファーム（確定版） | `D:\hobby\water_boarder\PIC12F18XX\i2c_wb_5V.c` / `i2c_wb_3V3.c`（+`gen_3v3.sh`） |
| Arduino ライブラリ | `D:\hobby\water_boarder\PIC12F18XX\WaterBoader\`（このフォルダ） |
| 既存 README/SPEC | 同上 `README.md` / `SPEC.md` |
| 参照統合（実証） | `D:\hobby\AquaController\`（lib/WaterBoader, sensors.cpp, config.h, state.h, main.cpp） |
| 全経緯DISPATCH | `D:\hobby\water_boarder\PIC12F18XX\DISPATCH.md` |
| DS全文 | `D:\hobby\water_boarder\PIC12F18XX\ds_1840.txt`(DS40001441F,397p) |
| XC8ビルド | `cd D:/hobby/water_boarder/PIC12F18XX && "C:/Program Files/Microchip/xc8/v4.00/bin/xc8-cc.exe" -mcpu=PIC12F1840 -mdfp="C:/Users/sinhex/.mchp_packs/Microchip/PIC12-16F1xxx_DFP/1.9.258/xc8" -O0 -o build/i2c_wb_5V.elf i2c_wb_5V.c`（3V3も同様） |
| PlatformIO(AquaController) | `~/.platformio/penv/Scripts/pio.exe run -e esp32-c3` |

## 8. 関連メモリ

`waterboader-pic12f1840`（本命・全実装）/ `waterboader-i2c-slave` / `waterboader-project` / `debug-regression-method` / `aqua-device-ops-quirks`。索引 = `MEMORY.md`。

## 9. 最初の一手（新スレッド）

1. 独立 repo ディレクトリを決めて `git init`、標準ライブラリ構成へ `src/` `examples/` `firmware/` 配置。
2. `library.properties` に加え `library.json`（PlatformIO）作成、semver `1.0.0`、keywords（capacitive, water-level, non-contact, PIC12F1840, I2C）。
3. README を §2 の章立てで起こす（§3-6 の数値をそのまま流用）。
4. （任意）電極を LCR 実測して §3 の絶対 pF を確定。
5. Example を arduino-cli / pio でコンパイルチェック→動作GIF/ログを README に貼る。
