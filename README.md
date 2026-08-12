# Main code for the 2026 REBUILT FRC season

The programmer knows what it is doing at all times. It knows this because it knows what it doesn't know, by subtracting what it knows, from what it doesn't, or what it doesn't, from what it does, whichever is greater, it obtains a difference, or deviation. The frontal-lobe sub-system uses deviations to generate corrective commands to write the code from a state where it is, to a state where it isn't, and arriving at a state where it wasn't, it now is. Consequently, the state where it is, is now the state that it wasn't, and it follows that the state where it was, is now the state that it isn't. In the event of the position that it is in is not the state that it wasn't, the system has required a variation. The variation being the difference between what the code is, and what it wasn't. If variation is considered to be a significant factor, it too, may be corrected by the frontal-lobe. However, the code must also know what state it was. The 3 AM energy drink scenario works as follows: Because a variation has modified some of the information the programmer has obtained, it is not sure just what it is, however it is sure what it isn't, within reason, and it knows what it was. It now subracts what it should be, from what it wasn't, or vice versa. By differentiating this from the algebraic sum of what it shouldn't be, and what it was. It is able to obtain a deviation, and a variation, which is called "grass".

## 現在の安全制約

- Intake actuator、Shooter actuator、Turretの通常位置制御は、limit switchまたはabsolute encoderによるhomingが実装されるまで出力を拒否します。Climberの通常動作も方向・limit・homing確定まで拒否します。これら5台は自動Hardware Self-Testでは動かさず、別途armした手動診断だけが、referenceを発行せず、選択した1台を一方向3%・最大0.35秒だけ動かします。
- PathPlanner autonomousは、wheel radius・module geometry・gearing・maximum speedをCAD/実測値へ統一するまでsafe-stopだけを返します。
- Feeder ID 32は過去の実機ログで約44 A・約0 rpmだったため、詰まり・機構・電源枝を点検して制御下で再試験するまで、通常のfeed/rejectとHardware Self-Testのmotion testを遮断します。
- すべてのSPARK/CTRE nonzero要求は、独立5 ms watchdogが監視する100 msのrobot-loop heartbeatがfreshで、かつ直前の`CommandScheduler.run()`が正常完了した間だけ受理します。scheduler完了なしでheartbeatだけを繰り返しても期限は延びません。CommandSchedulerまたはrobot loopが停止した場合はprocess-wide authorizationを先に失効し、全SPARKとswerveのfresh zero evidenceが揃うまでscheduler外で停止を再試行します。同じEnabled sessionでは自動再始動せず、Disabledで停止確認後にだけ再armします。

## Dashboard tuning

Swerveの`Drive/Output Scale`は現在`FIXED_CODE_CONFIGURATION`で、speed/rotationともに5%へ固定しています。旧`Speed Factor`/`Rotation Factor`の編集欄は値が実行経路へ届かなかったため除去しました。距離Alignmentはcamera geometry・対象tag・距離モデル・操作bindingが未確定なので、未完成commandを削除し`Drive/Alignment Availability=UNAVAILABLE_PENDING_TARGET_DISTANCE_MODEL_AND_BINDING`と表示します。

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
| Maintenance（Testのみ・fallbackなし） | Create + L1 / R1 | 選択済みmotorのnegative / positive 3%隔離診断pulse |

Red/Blueの競技画面には直近操作の`Operator Action State`と`Operator Action Reason`、全出力gateとglobal stop evidenceの`Output Safety`を表示します。`ACTIVE`は機構ごとのsoftware APIが要求を受け付けたか、Swerveではnative control APIが例外なく戻ったことを示します。Swerve requestの内部受付、CAN frameの送信、物理的な動作の証明ではありません。`STOPPED`はoperator actionが終了して停止要求を発行した状態で、controller出力のzero確認を意味しません。`BLOCKED`は未reference、既知Feeder stall、入力競合、release待ち、CAN healthなど、その操作で実際に判定した理由を表示します。全10操作の状態、出力CAN、依存CANはDiagnostics / Setupの`Operator Action Evidence`で確認できます。

Desktop simulationのSPARK modelは`RAW_COMMAND_ECHO_NO_PHYSICS_NO_REFERENCE`です。command到達、zero停止、fault時の拒否は検証しますが、機構の慣性・位置・電流・polarityを再現せず、homing/referenceや実機motion evidenceを生成しません。

手動motor診断はDisabledでTest modeを選び、Diagnostics / SetupでID30/32/34/35/38/39とNegative/Positiveをそれぞれexact-one選択し、対象と方向についてPhysical ClearanceとBrushless Motor Typeを確認してからfresh Armを立てます。ID32だけは物理詰まり・電源枝の点検確認も必須です。その後Test Enableし、Maintenance controllerを一度全releaseしてからCreateと、snapshot済み方向に対応するL1またはR1だけを保持します。driver fallbackはありません。1回のArmで1 pulseだけ実行し、開始前は全SPARK、終了後は対象（Climberでは両側）のzero-output evidenceを確認します。全出力は共通robot-loop heartbeatでscheduler停止時にもzeroへ移行し、ID32はこれに加えて独立した0.35秒・8 A watchdogでもzeroを継続要求します。途中releaseでもcommandは停止確認までrequirementsを保持し、外部cancel時だけ`STOP_REQUESTED`までを事実どおり表示します。Hardware Self-Test Armとの同時armは両方拒否します。

Hardware Self-TestはDisabledでTest modeを選んだ状態でArmし、その有効時間内にTest Enableします。

## 実機commissioning

現在コードが前提にしているCAN mapは、SPARK `30..39`、Pigeon `20`、CANcoder `40..43`、swerve steer/drive `50..57`です。これはソフトウェア設定値であり、実機配線の証明ではありません。設計図またはlive inventoryと一致するまでcalibrated autonomousと未homing位置機構は有効にしません。

再接続後はDisabledのまま次の順で確認します。

1. Driver StationとDiagnostics / Setupで全23 CAN IDが一意かつfreshであることを確認する。
2. Pigeonが現在値`20`か、旧生成値`49`かを実機inventoryで確定する。
3. Hardware Self-Testの`GLOBAL_START` stop barrierがCONFIRMEDになってから、homing済みまたは連続回転機構の低出力motion evidenceを採る。ID30/32/34/35/38/39は自動試験対象外。
4. Feeder ID32は物理詰まり・電源枝を点検後、Disabled TestでID32・方向・clearance・motor type・`Jam and Power Branch Inspected`を選択してArmし、MaintenanceのCreate+L1/R1で一度だけ3%再試験する。通常feed/rejectと自動HSTは結果に関係なくblockされたまま。
5. ID30/34/35/38/39は上記の手動診断で1台・一方向ずつpolarity evidenceを採る。ID30/38/39は診断後もreference扱いにせず、limit switchまたはabsolute referenceを実装してからhomingし、位置方向・soft limitを確認する。ID34/35は設計図で同期方式・limit・semantic directionを確定するまで通常climbを有効にしない。
6. Swerveのwheel radius、module位置、gear ratio、maximum speedをCAD/実測と一致させた後にautonomous calibration blockを解除する。
