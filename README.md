# nakalab_ultralytics_ros2

人物姿勢検出に加え，YOLO インスタンスセグメンテーションによる物体検出を提供する ROS 2 Humble 用パッケージ群です．

> [!IMPORTANT]
> 3D 出力は，マスク内の有効深度から求めた **可視表面の重心と寸法** です．物体全体の中心・裏面の形状・把持姿勢を保証する値ではありません．`object_poses` の姿勢は単位 quaternion であり，物体の向きの推定結果ではありません．

## 配置とビルド <a id="build"></a>

このワークスペースでは，コンテナ内の `~/colcon_ws/thirdparty/nakalab_ultralytics_ros2` を `~/colcon_ws/src/nakalab_ultralytics_ros2` に移動しました．ホストでは `g1_ws/src/nakalab_ultralytics_ros2` に保存され，元の `.git` も保持しています．親ワークスペースと独立した Git リポジトリです．

`depends.repos` の配置先も `src` に変更しています．既存 Docker イメージに残る旧配置は，`katana` の Compose マウントで `COLCON_IGNORE` を置いて探索対象外にしています．これは colcon の標準の除外方式です．イメージの再ビルドは不要です．[colcon の探索仕様](https://colcon.readthedocs.io/en/released/reference/discovery-arguments.html)

以下はホストの `g1_ws` ディレクトリから実行します．依存ライブラリとモデルは既存の `katana` 環境を利用します．

```bash
docker compose run --rm --name agy katana bash -ic \
  'colcon build --symlink-install --packages-select nakalab_ultralytics_interfaces nakalab_ultralytics_cpp nakalab_ultralytics_api nakalab_ultralytics_ros2'
```

移動前の生成物は，復元用に `build/nakalab_before_src_move/build` と `install/.nakalab_before_src_move` に退避しています．ソースの複製ではなく，通常のパッケージ探索対象にもなりません．

## 起動 <a id="launch"></a>

1. RGB・整列深度・RGB の `CameraInfo` と，出力座標系への TF を配信します．既定の入力は D455 の以下のトピックです．

   | 入力 | 既定トピック |
   | --- | --- |
   | RGB | `/head_camera/d455/color/image_raw` |
   | RGB に位置合わせ済みの深度 | `/head_camera/d455/aligned_depth_to_color/image_raw` |
   | RGB 内部パラメータ | `/head_camera/d455/color/camera_info` |

1. 検出ノードを起動します．CPU での例です．`run_detect:=true` で直ちにモデルをロードします．

   ```bash
   docker compose run --rm --name agy katana bash -ic \
     'ros2 launch nakalab_ultralytics_ros2 object_seg_pose.launch.py device:=cpu run_detect:=true'
   ```

1. GPU を使う場合は `device:=cuda` を指定します．検証した `katana` イメージには cuDNN 9.20 と 9.25 が同居し，通常の起動では `CUDNN_STATUS_SUBLIBRARY_VERSION_MISMATCH` が発生しました．次の指定で GPU 推論まで確認しています．環境変数はこのコマンドのプロセスにだけ適用されます．

   ```bash
   docker compose run --rm --name agy katana bash -ic \
     'LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH ros2 launch nakalab_ultralytics_ros2 object_seg_pose.launch.py device:=cuda run_detect:=true'
   ```

終了は起動端末の `Ctrl+C`，別端末からの明示停止は `docker stop agy` です．同名のコンテナを同時に起動しないでください．既存の人物検出 launch と bringup の構成は維持しています．

| launch 引数 | 既定値 | 意味 |
| --- | --- | --- |
| `model_path` | `/tmp/yolo26l-seg.pt` | 配置済みの segmentation 重み |
| `device` | `cuda` | Ultralytics の推論デバイス指定 |
| `run_detect` | `false` | 起動時にモデルをロードするか |
| `enable_3d` | `true` | C++ の深度変換ノードも起動するか |
| `ref_frame` | `base_link` | 3D 出力の座標系 |
| `use_sim_time` | `false` | `/clock` 使用時は入力ノード・API とともに `true` |
| `confidence` | `0.5` | 起動時の信頼度しきい値 |
| `imgsz` / `max_det` | `640` / `20` | 推論サイズ・最大検出数 |
| `max_rate_hz` | `10.0` | 推論タイマーの最大頻度 |
| `sync_tolerance_sec` | `0.1` | RGB と深度の許容時刻差，秒 |
| `color_image` / `depth_image` / `color_camera_info` | 上表 | 入力トピックの変更 |

実際の処理頻度は推論時間にも依存します．待ち画像は最新の 1 枚に限定します．画像のみの場合は `enable_3d:=false` とします．3D をカメラ座標系で取得する場合は `ref_frame` に RGB の光学座標系名を指定します．

モデルは `task=segment` を検証します．重みの自動ダウンロードは行いません．検出可能なクラスは重みの `names` に依存します．`retina_masks=True` により元画像サイズで返されたマスクを扱い，矩形領域へ切り出す際にも穴や離れた領域を維持します．[Ultralytics のセグメンテーション仕様](https://docs.ultralytics.com/tasks/segment/)

`object_seg_pose` ノードには追加で `class_names`（既定 `['*']`）と `publish_image`（既定 `true`）があります．推論対象を限定する場合は，単独ノードの起動パラメータでクラス名を渡せます．設定は起動時に読み取ります．

```bash
docker compose run --rm --name agy katana bash -ic \
  'ros2 run nakalab_ultralytics_ros2 object_seg_pose --ros-args -p device:=cpu -p run_detect:=true -p "class_names:=[cup, bottle]" -r color_image:=/head_camera/d455/color/image_raw'
```

## 出力と開始・停止 <a id="interfaces"></a>

名前はすべてノードの名前空間に対する相対名です．以下はルート名前空間での表記です．

| トピック・サービス | 型 | 内容 |
| --- | --- | --- |
| `/nu_ros2/object_seg_2d` | `ObjectSeg2DArray` | クラス，信頼度，矩形，マスク |
| `/nu_ros2/object_detect_image` | `sensor_msgs/Image` | 検出描画付き RGB 画像 |
| `/nu_ros2/object_seg_3d` | `ObjectSeg3DArray` | 有効性，理由，位置，寸法，深度点数 |
| `/nu_ros2/object_markers` | `visualization_msgs/MarkerArray` | 可視表面の軸平行ボックス |
| `/nu_ros2/object_poses` | `geometry_msgs/PoseArray` | 有効な可視表面重心の位置 |
| `/nu_ros2/detect_object` | `nakalab_ultralytics_interfaces/srv/Detect` | `run` と `confidence` による開始・停止 |

`ObjectSeg2DArray.header` と個々のマスクの header は元 RGB 画像の時刻・座標系です．`roi` は元画像における左上画素と幅・高さ，`mask` はその ROI 内だけの `mono8` 画像（0 または 255）です．`detection_id` はフレーム内の識別番号で，追跡 ID ではありません．検出ゼロ・停止・フレーム処理失敗時には空配列を配信します．停止時は元画像サイズを持たない空配列になる場合があります．

画像入力と描画画像は SensorData QoS，検出配列は reliable，depth 10 です．開始・停止には既存の `Detect.srv` を再利用し，人物検出のサービス名やメッセージは変更していません．

```python
import rclpy
from rclpy.node import Node
from nakalab_ultralytics_api.object_detector import ObjectDetector

rclpy.init()
node = Node('object_task')
detector = ObjectDetector(node)
try:
    result = detector.detect(timeout_sec=15.0, confidence=0.5,
                             class_names=['cup', 'bottle'], require_3d=True)
    if result is not None:
        for obj in result.objects:
            node.get_logger().info(
                f'{obj.class_name}: {obj.centroid}, frame={result.header.frame_id}')
finally:
    detector.close()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

`detect()` は開始要求より前の撮影時刻を除外し，3D では `valid=True` の結果だけを返します．`class_names` は API 側で返す結果の絞り込みです．推論対象の絞り込みにはノード側の同名パラメータを使います．既定では成功・タイムアウト・例外時に停止要求を送ります．この API 自身が node を spin するため，同じ node を別 executor で同時に spin しないでください．`demo_detect_object` もインストールします．

## 3D 変換の仕様 <a id="depth"></a>

RGB と深度を撮影時刻で同期し，両者と `CameraInfo` の画像寸法・光学座標系が一致することを確認します．RGB の撮影時刻に対応する TF だけを使用し，取得できない場合に最新 TF へ置き換えることはありません．

深度は `16UC1` の mm または `32FC1` の m に対応します．ゼロ・非有限値・範囲外の深度を除外し，マスク内深度の中央値と MAD（$\text{Median Absolute Deviation}$，中央絶対偏差）で外れ値を除去します．歪み係数は `plumb_bob`，`rational_polynomial`，`equidistant` に対応します．クロップ済み・binning 済みの `CameraInfo` は未対応です．[ROS の深度画像仕様 REP 118](https://www.ros.org/reps/rep-0118.html)

`ObjectSeg3DArray.header` は出力座標系，`source_header` は元 RGB 座標系です．両者の時刻は同じです．点群を出力座標系へ変換した後で，重心 `centroid`，軸平行ボックス中心 `box_center`，寸法 `size` を m 単位で計算します．深度・TF・入力整合性が不正な場合は `valid=False`，位置・寸法は NaN，`reason` に理由を設定します．

| 3D ノードの起動パラメータ | 既定値 |
| --- | --- |
| `min_depth_m` / `max_depth_m` | `0.1` / `6.0` |
| `pixel_stride` / `min_points` | `2` / `12` |
| `depth_outlier_floor_m` | `0.02` |
| `stale_timeout_sec` | `2.0` |

`stale_timeout_sec` は壁時計で測定します．結果更新が途絶えると空配列・空の位置配列・マーカー消去を配信します．同期キューの長さは 30 で，推論が入力に追いつかない場合は古い組を落とします．RViz 描画だけには最小厚さ 5 mm を設けますが，計測値 `size` は変更しません．平面の観測では寸法の一軸が 0 になる場合があります．

## 検証 <a id="tests"></a>

通常の回帰試験には既存の人物検出試験，マスク変換，モデル開始・停止，API の時刻・クラス・有効性フィルター，深度単位・エンディアン・欠損値・外れ値・歪み補正を含みます．現行コンテナでは古い pytest と追加済み anyio プラグインに互換性がないため，プラグインの自動読み込みを無効にして実行します．

```bash
docker compose run --rm --name agy katana bash -ic \
  'ROS_DOMAIN_ID=211 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test --packages-select nakalab_ultralytics_interfaces nakalab_ultralytics_cpp nakalab_ultralytics_api nakalab_ultralytics_ros2 --executor sequential --pytest-args -p no:cacheprovider'
```

`test/test_object_pipeline.py` はインストール済みノードを起動して通信を検証します．通常ドメインでは実行をスキップします．実モデル試験はローカル資産のパスを明示した場合だけ動き，画像・重みをダウンロードしません．下記は GPU での再実行コマンドです．`OBJECT_SEG_TEST_DEVICE=cpu` でも実行できます．

```bash
docker compose run --rm --name agy katana bash -ic \
  'export ROS_DOMAIN_ID=211 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 YOLO_OFFLINE=true;
   export OBJECT_SEG_TEST_MODEL=/tmp/yolo26l-seg.pt;
   export OBJECT_SEG_TEST_IMAGE=/home/gai/.local/lib/python3.10/site-packages/ultralytics/assets/bus.jpg;
   export OBJECT_SEG_TEST_DEVICE=cuda;
   LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH python3 -m pytest -s -q -p no:cacheprovider src/nakalab_ultralytics_ros2/test/test_object_pipeline.py'
```

結合試験は TF の回転・並進，無効深度，検出消去，入力停止，実モデルのマスク，開始・停止 API，子プロセスの正常終了を確認します．実モデル試験の深度は一様な 2 m の合成画像です．実カメラの測距精度・認識率・ロボットの把持成功率は，この試験の対象には含みません．

2026-10-07 の検証では，対象 4 パッケージと依存する `erasers_g1_tasks` のビルドが成功し，対象 4 パッケージの colcon 集計は 71 tests，0 errors，0 failures，10 skipped でした．実モデルを含む結合試験 2 件は CPU と GPU の両方で通過しました．

ワークスペース全体のビルドも試行しましたが，既存の `thirdparty/livox_ros_driver2` に `package.xml` がないため停止しました．同ディレクトリには `package_ROS1.xml` と `package_ROS2.xml` があります．今回の物体検出の変更では，この別パッケージの構成は変更していません．
