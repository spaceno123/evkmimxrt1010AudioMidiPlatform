# evkmimxrt1010AudioMidiPlatform
Usb Audio &amp; Midi Basic Platform on EVKMIMXRT1010

Audio &amp; Midi の基本的な作業プラットフォームの構築がこのリポジトリの目的です。

evkmimxrt1010_dev_composite_hid_audio_unified_freertos を元に改造しています。

変更箇所は以下の通り

・hid を Midi に置き換え<br>
・fs=48kHz のクロックを修正<br>
・Codec とのデータ転送単位を 32bit 化<br>
・Usb Audio のデータ転送単位を 24bit 化<br>
・Codec の In/Out 処理サイクルを 4fs に変更<br>
・Vector 領域を ITCM の先頭に移動<br>
・ファイルオブジェクト単位で実行コードを ITCM に配置<br>
・関数単位で実行コードを ITCM に配置<br>
・printf で float を有効化<br>

拡張分は以下の通り

・Debug Monitor を追加<br>
