# Main code for the 2026 REBUILT FRC season

The programmer knows what it is doing at all times. It knows this because it knows what it doesn't know, by subtracting what it knows, from what it doesn't, or what it doesn't, from what it does, whichever is greater, it obtains a difference, or deviation. The frontal-lobe sub-system uses deviations to generate corrective commands to write the code from a state where it is, to a state where it isn't, and arriving at a state where it wasn't, it now is. Consequently, the state where it is, is now the state that it wasn't, and it follows that the state where it was, is now the state that it isn't. In the event of the position that it is in is not the state that it wasn't, the system has required a variation. The variation being the difference between what the code is, and what it wasn't. If variation is considered to be a significant factor, it too, may be corrected by the frontal-lobe. However, the code must also know what state it was. The 3 AM energy drink scenario works as follows: Because a variation has modified some of the information the programmer has obtained, it is not sure just what it is, however it is sure what it isn't, within reason, and it knows what it was. It now subracts what it should be, from what it wasn't, or vice versa. By differentiating this from the algebraic sum of what it shouldn't be, and what it was. It is able to obtain a deviation, and a variation, which is called "grass".

## 現在の安全制約

- Intake actuator、Shooter actuator、Turretの位置制御は、limit switchまたはabsolute encoderによるhomingが実装されるまで出力を拒否します。
- PathPlanner autonomousは、wheel radius・module geometry・gearing・maximum speedをCAD/実測値へ統一するまでsafe-stopだけを返します。
- Feeder ID 32は過去の実機ログで約44 A・約0 rpmだったため、詰まり・機構・電源枝を点検して制御下で再試験するまで、通常のfeed/rejectとHardware Self-Testのmotion testを遮断します。

## Dashboard tuning

数値欄を書き換えただけでは実行値は変わりません。ロボットをDisabled、FMS未接続にしたうえで、対応するApplyをfalseからtrueへ切り替えてください。Applyはtrueのままでは再適用されません。次に適用するときは一度falseへ戻してからtrueへ切り替えます。

| 対象 | Apply key | 受付範囲 |
| --- | --- | --- |
| Intake | `Tuning/Intake/Apply` | PID 0..10、角度 0..10 deg、in 0..0.20、out -0.20..0 |
| Shooter | `Tuning/Shooter/Apply` | PID 0..10、100..1000 rpm、角度 0..5 deg |
| Turret | `Tuning/Turret/Apply` | PID 0..10 |
| Feeder | `Tuning/Feeder/Apply` | feed 0..0.20、reject -0.20..0 |
| Conveyor | `Tuning/Conveyor/Apply` | in 0..0.20、out -0.20..0 |

結果は対応する`Tuning/<name>/Status`に表示されます。PIDを変更するとcontroller configurationとposition continuityが失効するため、将来homingを実装した後は再homingが必要です。

`SPARK_HEALTH`の`SETPOINT_US=<last>/<max>`はREV setpoint JNIの直近・最大実行時間（microseconds）です。実機再接続時にDriver Stationのloop overrunとあわせて確認してください。

## 操作map

Teleopへ入った直後、controllerの再接続後、または機構healthの変化後は、全入力を一度neutral/releaseにしてから新しく操作します。同時に複数のintake-pathボタンを押した場合も、全releaseまで再始動しません。

| Controller | 操作 | 動作 |
| --- | --- | --- |
| Driver | sticks | swerve drive |
| Driver | L1 / R1 | intake / output |
| Driver | L2 / R2 | shooter rev / fire |
| Driver | Create / R3 / Touchpad | field seed / jump-bump / wheel lock |
| Operator | L1 | intake retract（controllerが無い場合Driver Square） |
| Maintenance | L1 | turret auto-aim（controllerが無い場合Driver Triangle） |
| Maintenance（未接続時のみDriverへfallback、Testのみ） | Options + Square/Cross/Circle/Triangle | climber left+/left-/right+/right- 0.35秒診断pulse |

Climber診断はTest Enabled、FMS未接続、DashboardのArmとBrushless Motor Type Verifiedが必要です。Hardware Self-TestはDisabledでTest modeを選んだ状態でArmし、その有効時間内にTest Enableします。

## 実機commissioning

現在コードが前提にしているCAN mapは、SPARK `30..39`、Pigeon `20`、CANcoder `40..43`、swerve steer/drive `50..57`です。これはソフトウェア設定値であり、実機配線の証明ではありません。設計図またはlive inventoryと一致するまでcalibrated autonomousと未homing位置機構は有効にしません。

再接続後はDisabledのまま次の順で確認します。

1. Driver StationとDiagnostics / Setupで全23 CAN IDが一意かつfreshであることを確認する。
2. Pigeonが現在値`20`か、旧生成値`49`かを実機inventoryで確定する。
3. Hardware Self-Testの`GLOBAL_START` stop barrierがCONFIRMEDになってから低出力motion evidenceを採る。
4. Feeder ID32は物理詰まり・電源枝を解消してから、compile-time controlled retestを明示的に有効化して3%だけ再試験する。
5. ID30/38/39はlimit switchまたはabsolute referenceを実装してからhomingし、位置方向・soft limitを確認する。
6. Swerveのwheel radius、module位置、gear ratio、maximum speedをCAD/実測と一致させた後にautonomous calibration blockを解除する。
