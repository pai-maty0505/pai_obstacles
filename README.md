# pai_obstacle

SO-101 pick-and-place with obstacle avoidance (MoveIt 2)

SO-101 アームが、ランダムに配置された障害物を避けながら、運搬物（荷物）を皿まで 3 分以内に運ぶ。

---

## 1. 環境構築方法

### 1.1 必要なもの

| 項目 | 内容 |
|---|---|
| OS | Ubuntu 24.04 |
| ROS 2 | Jazzy（Gazebo Harmonic は `ros-jazzy-ros-gz` で入る） |
| GPU | 学習に使う（RTX 3060 12 GB で確認）。なくても CPU で学習できるが遅い |
| 置き場所 | このリポジトリ `~/pai_obstacle`、SO-101 のワークスペース `~/so101_ws` |

### 1.2 ROS 2 Jazzy と使うパッケージ

ROS 2 Jazzy の apt のリポジトリを、[公式の手順](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)で登録してから入れる。

```bash
sudo apt update && sudo apt install -y ros-jazzy-desktop ros-jazzy-moveit ros-jazzy-pilz-industrial-motion-planner ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-xacro python3-colcon-common-extensions python3-numpy python3-scipy ffmpeg
```

`python3-numpy`・`python3-scipy` は、ROS 2 を使うスクリプト（システムの Python）が使う。`ffmpeg` は Gazebo の動画を作るのに使う。

### 1.3 このリポジトリ

```bash
git clone https://github.com/pai-maty0505/pai_obstacles.git ~/pai_obstacle
```

Gazebo の起動・撮影・判定の仕組み（4 章）と、SO-101 のパッチ・グリッパの部品を作るスクリプト（1.4 節）も、このリポジトリに入っている。

### 1.4 SO-101 の ROS 2 ワークスペース

[adoodevv/so101_ros2](https://github.com/adoodevv/so101_ros2) のコミット `0305e03` を `~/so101_ws/src/so101_ros2` に置き、[patches/so101_ros2_local.patch](patches/so101_ros2_local.patch) を当ててビルドする。パッチの内容は、Jazzy のパッケージ名に合わせて `pilz_industrial_motion_planner` に変えること、Gazebo のモデルだけ `base_link` をワールドに固定すること、グリッパを両側から挟む形に改造すること（レポートの 1.8 節）。

```bash
mkdir -p ~/so101_ws/src && cd ~/so101_ws/src && git clone https://github.com/adoodevv/so101_ros2.git
```

```bash
cd ~/so101_ws/src/so101_ros2 && git checkout 0305e03 && git apply ~/pai_obstacle/patches/so101_ros2_local.patch
```

```bash
source /opt/ros/jazzy/setup.bash && cd ~/so101_ws && colcon build
```

パッチを当てた後、左の指と手首の部品のメッシュを作ってから、もう一度ビルドする。

```bash
python3 ~/pai_obstacle/scripts/make_gripper_meshes.py
```

```bash
cd ~/so101_ws && colcon build --symlink-install --packages-select so101_description so101_moveit_config
```

### 1.5 学習用の Python（PyTorch）

学習（`train_mlp.py`）だけは、ROS 2 と別の Python で行う。例として Miniconda（`~/miniconda3`）に PyTorch 2.5.1 + CUDA 12.1 を入れる。

```bash
~/miniconda3/bin/pip install numpy
```

```bash
~/miniconda3/bin/pip install torch==2.5.1 --index-url https://download.pytorch.org/whl/cu121
```

### 1.6 確認

Gazebo・MoveIt 2・RViz が起動して準備が整うかを確かめる（初回の起動は約 30 秒）。物理エンジンは bullet-featherstone、ロボットの衝突形状は凸分解したもの（起動時に作る）を使う。

```bash
cd ~/pai_obstacle/gazebo && scripts/sim_start.sh
```

```bash
cd ~/pai_obstacle/gazebo && scripts/sim_check.sh
```

停止：

```bash
cd ~/pai_obstacle/gazebo && scripts/sim_stop.sh
```

起動時に次のエラーが出るが、使っていない機能のもので害はない：`No 3D sensor plugin(s) defined for octomap updates`、`ExecuteTaskSolutionCapability ... does not exist`、`Action server: /recognize_objects not available`。

---

## 2. 検討の前提と限界

- 半径はパン軸から測り、高さはベースの底面（台の上面）から測った。
- 衝突判定では、障害物を 1.5 cm 膨らませた。
- 関節角は格子状に、パン角は 0.04 rad 刻みでサンプリングした。
- 障害物の距離は保守的に近似した（平面への距離の最大値）。
- 障害物の向きは、割り当て 1 通りにつき 1 パターンしか試していない。
- 以上より、この検討は MoveIt 2 による衝突判定の代わりにはならない。数値は ±1 cm 程度の誤差を見込む。

---

## 3. 未決定事項

- 荷物と皿の配置範囲と、最低距離の具体的な値（教師データではレポートの 2.2 節の値を使っている）
- 荷物の初期の向き（yaw）をランダムにするか、その範囲（教師データでは、面をパッドの向きにそろえている）
- 皿の直径 4 cm が内径か外径か、縁の厚み（教師データでは外径 4 cm、縁の厚さ 3 mm として扱っている）
- パッドを 2 cm の荷物に合わせて作り直すか（レポートの 2.4 節）
- 荷物と障害物の位置をどう認識するか（カメラの種類と配置）
- レポートの 2.7 節の接触の例外を、ルール上どう扱うか
- 評価方法（試行回数、成功の判定、記録する指標：成功率・所要時間・衝突・リトライ回数）

---

## 4. 実行方法

学習した MLP が出す関節の角度で、Gazebo（物理シミュレーション）の SO-101 に荷物を運ばせる。学習済みの重みと推論した軌道はこのリポジトリに入れていないので、先に 5 章の手順で作る。

### 4.1 環境

| 項目 | 内容 |
|---|---|
| OS・ROS 2 | Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic、MoveIt 2、ros2_control（1 章） |
| SO-101 | `~/so101_ws`（1.4 節のパッチを当てたもの） |
| Gazebo の仕組み | v2 のものを [gazebo/](gazebo) に入れてある。起動（`gazebo/scripts/sim_start.sh`）、撮影・接触の記録・判定（`gazebo/scripts/run_episode_recorded.sh`）、字幕付きの動画、アームとグリッパの動かし方をそのまま使う |
| Python | ROS 2 を使うスクリプトはシステムの Python（`/usr/bin/python3`、numpy・scipy）。学習（`train_mlp.py`）だけは PyTorch の入った Python（例：`~/miniconda3/bin/python`、PyTorch 2.5.1 + CUDA 12.1） |

環境の作り方は 1 章。以下、このリポジトリを `~/pai_obstacle` に置いたものとする。

| ディレクトリ | 内容 |
|---|---|
| `scripts/` | お手本の生成・学習・確かめ・Gazebo での実行のスクリプト |
| `data/` | 姿勢（`poses.json`）。お手本・学習の結果もここに出る |
| `sim/models/` | 三角錐のメッシュ |
| `gazebo/` | Gazebo の起動・撮影・判定の仕組み（`scripts/`・`sim/`・`data/`）。記録は `gazebo/logs/` に出る |
| `patches/` | SO-101 の ROS 2 ワークスペースに当てるパッチ（1.4 節） |

ROS 2 を使うコマンドは、次を読み込んだ端末で実行する。

```bash
source /opt/ros/jazzy/setup.bash && source ~/so101_ws/install/setup.bash
```

### 4.2 Gazebo で動かす

Gazebo を起動する。

```bash
cd ~/pai_obstacle/gazebo && scripts/sim_start.sh
```

別の端末で、5 章で作った推論軌道のエピソードを、番号を指定して続けて実行する（撮影・接触の記録・判定・動画をまとめて行う）。

```bash
cd ~/pai_obstacle/gazebo && scripts/run_episode_recorded.sh --script ~/pai_obstacle/scripts/gazebo_v3.py -- --episodes ~/pai_obstacle/data/dataset_L17_G15_W2/mlp_s42/test_episodes_raw.jsonl --indices 136,824,646
```

| 引数（`--` の後ろ） | 内容 |
|---|---|
| `--episodes` | エピソードのファイル（`validate_mlp.py` が作る `test_episodes_raw.jsonl`） |
| `--indices` | 実行するエピソードの番号（カンマ区切り、この順に実行）。5.5 節の判定で当たらず、円柱の上を越えたものから選ぶ |
| `--clearance` | MoveIt で確かめるときの障害物の余裕 [m]。既定 0.008（レポートの 1.10 節） |
| `--preflight-only` | MoveIt で確かめるだけで、Gazebo では動かさない |

1 エピソードの流れ（[scripts/gazebo_v3.py](scripts/gazebo_v3.py)）：

1. 準備：高い姿勢に上がって次の荷物の上へ回る → 障害物 3 個・皿・荷物をエピソードの配置どおりに Gazebo に置き直す（三角錐は [sim/models/pyramid_v3.stl](sim/models/pyramid_v3.stl)）→ MoveIt で、荷物の上へ下りる区間とエピソードの全区間を余裕 8 mm で確かめる
2. 本番：真下に下ろす → 左右のパッドで挟む（指 0.096 rad、保持は摩擦だけ）→ 持ち上げる → **MLP が推論した軌道で運ぶ**（いちばん大きく動く関節が 0.4 rad/s の一定の速さ）→ 皿に下ろす → 指を開いて離す → 真上に上げる
3. 記録：運ぶ間の荷物のずれ（指から見た位置の変化）、置いた位置の皿の中心からのずれ、傾き、エピソードの長さ

成功の条件は、運ぶ間のずれ・置いた位置のずれがともに 10 mm 未満、傾きが 5° 未満で、不許可の接触・めり込みがないこと（荷物に触れてよいのは、左右のパッド・つかむ前の床・皿だけ）。

記録は `~/pai_obstacle/gazebo/logs/runs/<日時>_<引数>/` に保存される（`summary.txt`：判定のまとめ、`run.log`、`contacts.txt`：接触の記録、`video.mp4` とエピソードごとに切り出した動画）。

---

## 5. 学習方法

MoveIt 2 で障害物を避ける関節軌道（お手本）を作り、運ぶ区間の関節の角度を真似る MLP を学習する。モデルはランダムな初期値から学習する。

### 5.1 全体の流れ

| 手順 | スクリプト | Python | 出力 |
|---|---|---|---|
| 1. 姿勢の計算（省略できる） | `compute_poses.py` | システム | `data/poses.json`（このリポジトリに入れてある） |
| 2. お手本の生成 | `generate_parallel.py`（作業プロセスは `generate_teacher_v2_clear.py`） | システム | `data/dataset_L17_G15_W2/` |
| 3. MLP の学習 | `train_mlp.py` | PyTorch | `data/dataset_L17_G15_W2/mlp_s42/` |
| 4. 推論軌道の確かめ | `validate_mlp.py`（中で `validate_dataset.py` を使う） | システム | `data/dataset_L17_G15_W2/mlp_s42/` |
| 5. Gazebo で動かす | `gazebo_v3.py` | システム | 4 章 |

### 5.2 姿勢の計算

つかむ・置く区間のもとになる姿勢を `data/poses.json` に保存する。このリポジトリに入れてあるので、作り直すときだけ行う。`move_group` を 1 つ起動し、別の端末で（同じ `ROS_DOMAIN_ID`）実行する。

```bash
ROS_DOMAIN_ID=61 ros2 launch so101_moveit_config move_group.launch.py use_rviz:=false use_sim_time:=false use_camera:=false
```

```bash
cd ~/pai_obstacle && ROS_DOMAIN_ID=61 /usr/bin/python3 scripts/compute_poses.py
```

### 5.3 お手本の生成

Gazebo は使わず、MoveIt 2 の `move_group` だけで作る。`ROS_DOMAIN_ID` を分けて `move_group` を複数起動し、それぞれに作業プロセスを 1 つずつつないで並列に作る。エピソードの番号ごとに乱数の種を決めるので、同じ番号なら同じ配置になる。

**配置（1 エピソードごとにランダム）**

| 項目 | 値 |
|---|---|
| 範囲 | 回転軸から 20 cm 以内、`shoulder_pan` −1.5〜1.5 rad（土台の前の半円） |
| 荷物・皿 | 回転軸から 12〜20 cm。両端に置き、どちらに荷物を置くかもランダム |
| 障害物 | 3 種類を荷物と皿の間の角度に 1 個ずつ。向きもランダム。直方体は中心を 17.5 cm 以上（手前をハンドが通れるように） |
| 間隔 | 縁と縁（外接円で測る）が、障害物どうし 8 cm 以上、それ以外 5 cm 以上。障害物は土台からも 5 cm 以上 |

**運ぶ区間の作り方**

1. 道路網を作る。候補点は、作業範囲の格子（1.5 cm 間隔、回転軸から 9〜24 cm）、障害物のまわり（16 方向 × 3 距離）、円柱・三角錐の真上、荷物・皿の真上。高さは荷物の中心で 4〜20 cm を 1 cm おきの 17 層（高いところは指を傾ける）。各点は IK で関節の角度にする。
2. 当たらない点を、関節空間で近い 12 点と辺で結ぶ。
3. 荷物の真上から皿の真上まで、**経路の損 = 道のり ×（1 + 2 × 近さの損）** が最も小さい経路を探す。近さの損は、荷物とハンドの代表点（7 点）が障害物から 4 cm 以内に近づくほど大きい（0〜1）。円柱の上の点を必ず 1 つ通る（レポートの 1.4 節）。
4. 近道の処理とジグザグを取る処理をして、50 点に取り直す。
5. 全区間を、いちばん大きく動く関節で 0.01 rad ごとに衝突判定する（障害物の余裕 15 mm、持っている荷物を含む）。通り方（円柱は上を越える、直方体は手前を通る）と遠回りしないことを確かめ、満たさない配置は引き直す。

```bash
cd ~/pai_obstacle && PAI_LAYERS=17 PAI_GRID_STEP=0.015 PAI_CLEAR_WEIGHT=2 /usr/bin/python3 scripts/generate_parallel.py --episodes 1000 --workers 6 --version dataset_L17_G15_W2 --generator generate_teacher_v2_clear
```

`move_group` は自動で起動・停止する。途中で止めても、同じコマンドで続きから作る。

**出力（`data/dataset_L17_G15_W2/`）**

| ファイル | 内容 |
|---|---|
| `episodes.jsonl` | 1 行 = 1 エピソード。配置、全区間の関節の列（区間名・5 関節・指の角度）、運ぶ区間 |
| `transport.csv` | 運ぶ区間。1 行 = 1 点（50 点 × エピソード数） |
| `transport_{train,val,test}.csv` | 8：1：1 で分けたもの（1000 件で train 802 / val 99 / test 99） |
| `summary.csv`・`meta.json` | エピソードごとのまとめと、条件・集計 |

### 5.4 MLP の学習

| 項目 | 内容 |
|---|---|
| サンプル | 運ぶ区間の 1 点（50 点 × エピソード） |
| 入力（21） | 配置の特徴 20（`forward`、荷物・皿の `r`・`pan`、形ごとの `r`・`pan`・`yaw_sin`・`yaw_cos`・`order`）＋ 進行度 `progress`（0.0 = 荷物の真上、1.0 = 皿の真上） |
| 出力（5） | 5 関節の角度 |
| モデル | 21 → 256 → 256 → 256 → 5（ReLU、ドロップアウト 0.2）。入力・出力は train の平均と標準偏差で標準化 |
| 学習 | AdamW（学習率 1e-3、weight decay 1e-3）、MSE、バッチ 512。200 エポック良くならなければ終了 |
| 残す重み | 検証の精度（5 関節すべての誤差が 3° 以内の点の割合）が最大のとき（`--select acc`） |
| 乱数の種 | 42（`--seed`） |

```bash
cd ~/pai_obstacle && ~/miniconda3/bin/python scripts/train_mlp.py --version dataset_L17_G15_W2 --only-shortest --hidden 256,256,256 --dropout 0.2 --weight-decay 1e-3 --select acc --seed 42 --out data/dataset_L17_G15_W2/mlp_s42
```

出力は `model.pt`（重み・入出力の列・標準化の値・構成）、`history.csv`（エポックごとの損失）、`stats.json`（条件と test の誤差）、`test_predictions.jsonl`（test の各エピソードの推論軌道）。RTX 3060 で約 1.5 分（457 エポック、残した重みはエポック 257）。

### 5.5 推論軌道の確かめ

test の各エピソードの運ぶ区間を推論軌道に差し替え、つかむ・置く区間とつないで、お手本と同じ方法で全区間を衝突判定する（余裕 8 mm）。`move_group` は自動で起動する。

```bash
cd ~/pai_obstacle && /usr/bin/python3 scripts/validate_mlp.py --version dataset_L17_G15_W2 --mlp mlp_s42 --clearance 0.008
```

| ファイル | 内容 |
|---|---|
| `test_episodes_raw.jsonl` | 推論軌道をそのまま使ったエピソード（4 章の `--episodes` に渡す） |
| `test_episodes_pinned.jsonl` | 始点・終点を荷物・皿の真上に合わせて直したもの |
| `mlp_{raw,pinned}_8mm.csv` | エピソードごとの衝突判定（`safe` が 1 なら当たらない） |
| `mlp_metrics.json` | 当たらない件数、円柱の上を越えた件数、お手本からのずれ |

4 章で動かすエピソードは、`mlp_raw_8mm.csv` の `safe` が 1 で、`mlp_metrics.json` の `raw` で円柱の上を越えたものから選ぶ。
