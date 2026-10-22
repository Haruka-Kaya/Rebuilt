# 走行速度をDashboardで変更する

この変更を一度ロボットへデプロイした後は、速度調整のたびに再デプロイする必要はありません。
Driver Stationとロボットが接続されている状態で、DashboardのNetworkTablesから次の数値を編集します。
Elasticなどでは、対応する数値入力ウィジェットを追加してください。

| NetworkTablesのキー | 初期値 | 設定範囲 | 反映時点 |
| --- | ---: | --- | --- |
| `/SmartDashboard/Drive/MaxSpeedMps` | 10.24 | 0〜10.24 m/s | Teleop走行の次の制御周期 |
| `/SmartDashboard/Drive/MaxAngularRateRadPerSec` | 約4.712 | 0〜約4.712 rad/s | Teleop旋回の次の制御周期 |
| `/SmartDashboard/Drive/AutoSpeedMps` | 0.5 | 0〜2.0 m/s | 次のAuto開始時 |

例えばMaxSpeedMpsを2.0にすると、Teleopのスティック最大入力が2.0 m/sの指令になります。
旋回の1 rad/sは約57.3度/秒です。設定を0にすると、その種類の走行指令を0にします。
設定はNetworkTablesの永続化機能で保存され、プログラム再起動後も保持されます。
範囲外の値は範囲内へ補正し、NaNや無限大は初期値へ戻します。
これらは速度指令の設定です。実際の速度はタイヤ寸法、ギア比、負荷、電圧に依存します。

## Autoは専用の設定で5秒間前進する

Autoではロボットの前方向へ指定速度で5秒間走行し、その後は全モジュールの走行・旋回モーターへNeutralOutを送ります。
Teleop用の不感帯はAutoに適用しません。Auto開始後にAutoSpeedMpsを編集した場合は、次のAutoで反映します。
Autoを中断した場合も同じNeutralOutへ切り替えます。Disabled時と、Auto/Testで通常の走行コマンドが待機する場合も同様です。
Phoenix 6のIdleはモジュール状態を変更しないため、出力解除には使用しません。
NeutralOutは設定済みのBrake/Coastに従います。車輪の即時停止や停止距離を保証するものではありません。
シミュレーションでは制御モードと出力電圧を確認します。実機ではCAN負荷・通信断時の動作と停止距離を別途確認してください。

手動走行、Aのブレーキ、左バンパーの方向リセットはTeleopで有効です。
Auto中のこれらのボタン操作でAutoを中断しません。
SysIdのBack/Start＋X/YはTestモードで有効です。Testではスティックによる通常走行を行いません。

値が反映されない場合はキーの綴り、ロボットとの接続、Driver Stationのモードを確認してください。
設定変更と制御の切替はシミュレーションで検証済みです。実機への反映後は速度と前進方向を確認してください。
