# VadModel

## logger

### 方針

- VadModelはROSに依存させたくない．
- しかし，VadModelのlogは，RCLCPP_INFOを使用したい．
- testでは，RCLCPP_INFOではなく，testしやすいloggerを使用したい．

### 設計

- VadModel用のloggerのclassを作成する(`VadModelLogger`)
- このclassがVadModel用のloggerのinterfaceを規定する．
- loggerを追加したい人は，`VadModelLogger`を継承し，このinterfaceに沿うように，loggerを実装する．

### 設計の選択肢

#### どのように型を指定するか

1. requiresで指定
   - C++20以上でないと使用できないが，環境はC++17であるので不採用
2. templateとstatic_assertで指定
   - これを採用．
   - `VadModelLogger`を継承していないloggerのみ使用できるようにする．
3. SFINAEで指定
   - エラーメッセージがわかりにくいとgeminiに言われたのでやめておく
   - 以下，参考資料
     - <https://cpprefjp.github.io/lang/cpp11/sfinae_expressions.html>
     - <https://teratail.com/questions/251976>
