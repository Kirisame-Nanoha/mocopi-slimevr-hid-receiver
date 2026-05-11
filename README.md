# mocopi-slimevr-hid-receiver

mocopiセンサーをSlimeVR Server上のDIYトラッカーとして扱うための、非公式自作データレシーバーファームウェアです。

XIAO nRF52840をPCに接続し、mocopiセンサーのデータをBluetooth Low Energyで受信して、SlimeVR Server向けのUSB HIDトラッカー入力として利用できるようにします。

スマホアプリや常駐ブリッジアプリを使わず、XIAO nRF52840単体でmocopiセンサーをSlimeVR Serverへ接続することを目的としています。

## 概要


主な動作は以下です。

- mocopiセンサーをBLEで検出・接続する
- mocopiセンサーから姿勢データを受信する
- 受信したデータをSlimeVR Server向けUSB HID形式に変換する
- PCからはSlimeVR HIDレシーバーとして認識される
- SlimeVR Server上ではDIYトラッカーとして利用できる

現在は、6個のmocopiセンサーを使用する構成を想定しています。

想定部位は以下です。

- 腰
- 胸
- 左膝
- 左足
- 右膝
- 右足

## 特徴

- スマホアプリ不要
- 常駐ブリッジアプリ不要
- XIAO nRF52840単体でBLE受信とUSB HID出力を実行
- SlimeVR Server上でDIYトラッカーとして認識
- mocopiセンサーのMACアドレスを自動検出
- バッテリー残量表示に対応

## 必要なもの

- Seeed Studio XIAO nRF52840
- 通信用USB Type-Cケーブル
- mocopiセンサー 6個
- SlimeVR Serverが動作するPC
- `firmware.uf2`

任意で、XIAO nRF52840用のケースを使用してください。

## 書き込み方法

1. XIAO nRF52840をPCにUSB接続する
2. UF2ブートローダーのストレージを表示する
3. `Firmware.uf2` をUF2ストレージへドラッグ＆ドロップする
4. 自動的に再起動したら完了

UF2ストレージが表示されない場合は、PCにUSB接続した状態でリセットボタンを素早く2回押してください。

コピーがうまくできない場合や、コピー後も再度UF2ストレージとして表示される場合は、SoftDeviceが古い可能性があります。

UF2ストレージ内の `INFO_UF2.TXT` を開き、version が `7.3.0` より古い場合は、先に以下を書き込んでください。
s140_nrf52_7.3.0_softdevice.uf2

## 注意事項とお願い

・このファームウェアは、ChatGPT-5.5 Thinkingを利用しながら作成した、いわゆるバイブコーディングによる非公式ファームウェアです。コードの内容や動作について十分に確認したうえで使用してください。

・本ファームウェアは、個人利用・検証目的で作成したものです。全ての環境での動作を保証するものではありません。

・作成および使用は、個人利用の範囲で、必ず自己責任で行ってください。使用により、mocopiセンサー本体、XIAO nRF52840、PC、その他機器の破損、データ損失、怪我、トラブル、規約違反、第三者との紛争等が発生した場合でも、作者は一切の責任を負いません。

・本ファームウェアは、Sony株式会社、mocopiチーム、SlimeVR公式とは一切関係のない非公式プロジェクトです。Sony株式会社、mocopiチーム、SlimeVR公式への問い合わせは行わないでください。

・本ファームウェアは、SlimeVR Serverでの個人利用・検証を目的として作成しています。mocopi PC app、mocopi VR app、XYN Motion Studio、公式Sensor Data Receiverの代替品ではありません。

・本ファームウェアを、公式製品・公式サービス・公式レシーバーであるかのように表示、販売、配布、宣伝することは禁止します。

・本ファームウェアを使用した書き込み済みデバイスの販売、商用利用、第三者への有償提供は行わないでください。

・本ファームウェアの使用により、mocopiセンサーの保証や公式サポートが受けられなくなる可能性があります。あらかじめご了承ください。

## License

This project is licensed under the MIT License. See [LICENSE](./LICENSE).

## Acknowledgements

This project was created with reference to the following projects:

- [Kirisame-Nanoha/SlimeVR-Multi-Bridge](https://github.com/Kirisame-Nanoha/SlimeVR-Multi-Bridge)
- [verylowfreq/slimevr-hiddongle-espnow](https://github.com/verylowfreq/slimevr-hiddongle-espnow)
- [moslime/moslime](https://github.com/moslime/moslime)

This project is not affiliated with, endorsed by, or supported by Sony Corporation, SlimeVR, or the MoSlime developers.
mocopi is a trademark of Sony Corporation.
SlimeVR is a trademark of its respective owners.
