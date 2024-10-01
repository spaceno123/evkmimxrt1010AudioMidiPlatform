# evkmimxrt1010AudioMidiPlatform
Usb Audio &amp; Midi Basic Platform on EVKMIMXRT1010

Audio Midi の基本的な作業プラットフォームの構築がこのリポジトリの目的です。

evkmimxrt1010_dev_composite_hid_audio_unified_freertos を元に改造しています。

変更箇所は以下の通り

・hid を Midi に置き換え
・fs=48kHz のクロックを修正
・Codec とのデータ転送単位を 32bit 化
・Usb Audio のデータ転送単位を 24bit 化
・Codec の In/Out 処理サイクルを 4fs に変更
・Vector 領域を ITCM の先頭に移動
・ファイルオブジェクト単位で実行コードを ITCM に配置
・関数単位で実行コードを ITCM に配置
・printf で float を有効化

拡張分は以下の通り

・Debug Monitor を追加

