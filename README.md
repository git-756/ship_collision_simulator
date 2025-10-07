# 🚢 Ship Collision Simulator

**Pythonと物理エンジンPyBulletで構築する、船舶衝突の物理シミュレーター**

---

![Ship Collision Simulation GIF](https://example.com/your-simulation.gif)

## 概要 (Overview)

このプロジェクトは、「海上で一定速度で進む船が、緩衝材（ダンパー）を備え付けた壁に衝突した際の衝撃」をシミュレートするためのアプリケーションです。

単なる剛体の衝突計算だけでなく、船が海に浮いている状態（浮力・水の抵抗）や、船自身の推力、そして衝突面の衝撃吸収材（レクタングラーダンパー）の物理的特性（ばね・減衰）を考慮した、より現実に近いシミュレーションを目指しています。

開発の過程や詳細な解説は、こちらのブログ記事でも紹介しています。
[https://samurai-human-go.com/python-pybullet-physics-simulation-story/]

---

## ✨ 特徴 (Features)

* **物理ベースの衝突計算**: 物理エンジンPyBulletによるリアルな剛体ダイナミクス。
* **海洋環境の再現**: 浮力、水の抵抗、船の推力を考慮し、一定速度を維持する挙動をシミュレート。
* **衝撃吸収機構のモデル化**: `contactStiffness` と `contactDamping` パラメータを用いたスプリング・ダンパー機構の再現。
* **リアルタイム可視化**: PySide6によるGUIで、シミュレーションの様子をリアルタイムに確認可能。
* **エネルギー損失の算出**: 衝突によって失われた運動エネルギーを計算し、コンソールとGUIに表示。

---

## 🛠️ 技術スタック (Tech Stack)

* **シミュレーション**: PyBullet
* **GUI**: PySide6 (Qt for Python)
* **数値計算**: NumPy
* **パッケージ管理**: Rye

---

## 🚀 セットアップと実行方法 (Installation & Usage)

このプロジェクトは[Rye](https://rye-up.com/)によるパッケージ管理を前提としています。

1.  **リポジトリをクローン:**
    ```bash
    git clone https://github.com/git-756/ship_collision_simulator.git
    cd ship_collision_simulator
    ```

2.  **依存関係をインストール:**
    ```bash
    rye sync
    ```

3.  **シミュレーションを実行:**
    ```bash
    rye run python -m src.ship_collision_simulator.main
    ```

実行後、ウィンドウが表示されるので、「シミュレーション開始」ボタンをクリックしてください。

---

## 🔧 パラメータの調整 (Parameter Tuning)

シミュレーションの挙動は `src/ship_collision_simulator/simulation.py` 内のパラメータを変更することで調整できます。
特に、レクタングラーダンパーの性能をシミュレートしている以下の値を変更することで、衝撃吸収の効果がどのように変わるかを実験できます。

```python
# p.changeDynamics(self.boat_id, -1, ...) の中のパラメータ

contactStiffness=20000, # ダンパーの硬さ（ばね定数）。大きいほど硬くなる。
contactDamping=1000     # ダンパーの粘り気（減衰係数）。大きいほど衝撃を吸収し、反発が収まる。
```

---

## 🚧 今後の展望 (Future Work)

* [ ] ダンパーのパラメータをGUIから動的に変更できる機能
* [ ] 衝突時の最大加速度のグラフ表示
* [ ] 船や壁の形状をより複雑なモデルに変更する
* [ ] 波の影響を追加する

---

## 📜 ライセンス

このプロジェクトは **MIT License** のもとで公開されています。ライセンスの全文については、[LICENSE](LICENSE) ファイルをご覧ください。

また、このプロジェクトはサードパーティ製のライブラリを利用しています。これらのライブラリのライセンス情報については、[NOTICE.md](NOTICE.md) ファイルに記載しています。

## 作成者
[Samurai-Human-Go](https://samurai-human-go.com/%e9%81%8b%e5%96%b6%e8%80%85%e6%83%85%e5%a0%b1/)

**ブログ記事**: [【PyBullet開発奮闘記】Pythonで船の衝突シミュレーションを自作！「空飛ぶ船」と戦った全記録](https://samurai-human-go.com/python-pybullet-physics-simulation-story/)