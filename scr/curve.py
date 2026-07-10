import math


class Segment:
    """node_a と node_b をつなぐ1区間、または交差点内の専用フィレット。

    controls に0〜2個の制御点を渡すことで、直線・2次ベジエ・3次ベジエの
    どれでも表現できる(De Casteljauのアルゴリズムで統一的に計算する)。

    移動を「距離(ワールド距離単位)」で進められるように、あらかじめ
    弧長のテーブル(サンプリング)を作っておく。これにより、
    直線でも曲線でも常に同じ速さ(distance / frame)で進めて、
    かつ曲線区間では向き(tangent方向)が連続的に変化する。

    【進行方向の扱いについて】
    point_and_heading はノードIDでの判定を一切行わない。進行方向は
    reversed(bool)という、呼び出し側があらかじめ確定させたフラグ
    だけで決める。node_a/node_b はあくまで表示・デバッグ用のラベル
    であり、内部の向き判定には使わない。
    これは意図的な設計で、Segment/CornerSegmentを繋いだ複合カーブ
    (curve.CornerSegment、movement._build_chained_corner参照)では、
    ループするルート(同じノードIDを2回通る経路)を組み立てると、
    同一のノードIDが1本の経路の中に複数回登場し得る。もし向きの
    判定をノードIDの一致で行っていると、この再登場によって判定が
    衝突し、進行方向を取り違えてしまう(過去に実際に発生した不具合)。
    reversedフラグは、legや区間を組み立てたその場で構造的に
    (IDの再照合なしに)確定させて持ち回るため、同じIDが何度
    登場しても混同が起こらない。
    """

    SAMPLES = 24

    def __init__(self, node_a, node_b, pos_a, pos_b, controls=None):
        self.node_a = node_a
        self.node_b = node_b
        self.pos_a = pos_a
        self.pos_b = pos_b

        if controls is None:
            controls = []
        elif isinstance(controls, tuple) and len(controls) == 2 and all(
            isinstance(v, (int, float)) for v in controls
        ):
            # 制御点が1個だけ (x, y) の形で渡された場合、リストに包む
            controls = [controls]
        self.controls = list(controls)  # 0, 1, 2個の制御点

        self._table = self._build_length_table()
        self.length = self._table[-1][1]

    def _control_points(self):
        return [self.pos_a] + self.controls + [self.pos_b]

    @staticmethod
    def _de_casteljau(points, t):
        """De Casteljauのアルゴリズム。制御点の数に関わらず使える。"""
        pts = list(points)
        while len(pts) > 1:
            pts = [
                (
                    pts[i][0] * (1 - t) + pts[i + 1][0] * t,
                    pts[i][1] * (1 - t) + pts[i + 1][1] * t,
                )
                for i in range(len(pts) - 1)
            ]
        return pts[0]

    def point_at(self, t):
        return self._de_casteljau(self._control_points(), t)

    def tangent_at(self, t):
        """A->B方向に進んだときの向き(度)"""
        pts = self._control_points()
        deriv_pts = [
            (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)
        ]
        dx, dy = self._de_casteljau(deriv_pts, t)
        return math.degrees(math.atan2(dy, dx))

    def _build_length_table(self):
        table = [(0.0, 0.0)]
        prev = self.point_at(0.0)
        total = 0.0
        for i in range(1, self.SAMPLES + 1):
            t = i / self.SAMPLES
            p = self.point_at(t)
            total += math.hypot(p[0] - prev[0], p[1] - prev[1])
            table.append((t, total))
            prev = p
        return table

    def _t_for_distance(self, distance):
        """A地点からの弧長distanceに対応するtを、テーブルから線形補間で求める"""
        distance = max(0.0, min(distance, self.length))
        for i in range(1, len(self._table)):
            t0, d0 = self._table[i - 1]
            t1, d1 = self._table[i]
            if distance <= d1:
                if d1 == d0:
                    return t1
                ratio = (distance - d0) / (d1 - d0)
                return t0 + (t1 - t0) * ratio
        return 1.0

    # --- 移動で使う、向き付きの計算 ---

    def point_and_heading(self, distance, reversed=False):
        """node_a を起点に distance(ワールド距離単位)だけ進んだ
        位置と、その位置での向き(度)を返す。
        reversed=True の場合は node_b を起点に逆方向へたどる。

        向きの判定はこの reversed フラグだけで行い、ノードIDの
        一致判定は一切使わない(クラスdocstring参照)。"""
        if not reversed:
            t = self._t_for_distance(distance)
            return self.point_at(t), self.tangent_at(t)

        # node_b から node_a 方向へ進む場合
        t = self._t_for_distance(self.length - distance)
        heading = (self.tangent_at(t) + 180) % 360
        return self.point_at(t), heading

    def sample_points(self, count=20):
        """描画用に、A->B方向でcount+1点をサンプリングして返す"""
        return [self.point_at(i / count) for i in range(count + 1)]


class CornerSegment:
    """交差点X を挟んで Y -> X -> Z と進む経路を、1本の連続した区間として
    表す。

    XからYの手前 t の地点までは通常のY-X区間、
    その地点からXの先 s の地点(Z方向)までは制御点alphaの2次ベジエ、
    そこから先は通常のX-Z区間、という3つを繋げたもの。

    Segment と同じインターフェース(length, point_and_heading,
    sample_points)を持つので、そのまま使い回せる。

    【edge_yx / edge_xz と yx_reversed / xz_reversed について】
    edge_yx・edge_xz は「YからXへ至る区間」「XからZへ至る区間」を表す
    Segment または CornerSegment(チェーンされた複合カーブでもよい)。
    yx_reversed は「edge_yx.point_and_heading(d, reversed=yx_reversed)
    を呼ぶと、Y起点でY->X方向にdだけ進んだ位置が返る」ことを保証する
    フラグで、呼び出し側(graph._build_corner_segments、
    movement._build_chained_corner)が、edge_yx を用意した時点で
    構造的に(ノードIDの再照合なしに)確定させて渡す。xz_reversedも
    同様にedge_xzについて。

    これにより、CornerSegmentをいくつ繋いでチェーンしても、内部の
    向き判定が一切ノードIDに依存しなくなる。ルートが同じノードを
    2回以上通る(ループする)場合でも、chain中に同じIDが再登場する
    ことによる進行方向の取り違えが原理的に起こらない。
    node_a/node_b/node_x は表示・デバッグ用のラベルとしてのみ保持し、
    内部の計算では参照しない。
    """

    def __init__(self, edge_yx, edge_xz, yx_reversed, xz_reversed, t, s, alpha,
                 node_a=None, node_b=None, node_x=None):
        # 表示・デバッグ用のラベル(向き判定には使わない)
        self.node_a = node_a
        self.node_b = node_b
        self.node_x = node_x

        self.edge_yx = edge_yx
        self.edge_xz = edge_xz
        self.yx_reversed = yx_reversed
        self.xz_reversed = xz_reversed
        self.t = t
        self.s = s

        # Y-X区間のうち、Xの手前tより前の部分だけを使う
        self.leg1_length = max(0.0, edge_yx.length - t)

        # フィレットの両端点(Y側・Z側それぞれの、Xからのオフセット地点)
        # p1: Y起点でY->X方向に(edge_yx.length - t)進んだ位置 = Xの手前t
        # p2: X起点でX->Z方向にs進んだ位置 = Xの先s
        p1, _ = edge_yx.point_and_heading(edge_yx.length - t, reversed=yx_reversed)
        p2, _ = edge_xz.point_and_heading(s, reversed=xz_reversed)
        self.fillet = Segment("_corner_in", "_corner_out", p1, p2, alpha)

        # X-Z区間のうち、Xの先sより後ろの部分だけを使う
        self.leg3_length = max(0.0, edge_xz.length - s)

        self.length = self.leg1_length + self.fillet.length + self.leg3_length

    def _forward(self, distance):
        """Y(edge_yx側の起点)を起点に距離distance進んだ位置と向きを返す"""
        distance = max(0.0, min(distance, self.length))

        if distance <= self.leg1_length:
            return self.edge_yx.point_and_heading(distance, reversed=self.yx_reversed)

        distance -= self.leg1_length
        if distance <= self.fillet.length:
            # fillet は "_corner_in"(Y側)->"_corner_out"(Z側)の固定順で
            # 作った専用のSegmentなので、常にreversed=Falseで正しい
            return self.fillet.point_and_heading(distance, reversed=False)

        distance -= self.fillet.length
        return self.edge_xz.point_and_heading(self.s + distance, reversed=self.xz_reversed)

    def point_and_heading(self, distance, reversed=False):
        if not reversed:
            return self._forward(distance)
        pos, heading = self._forward(self.length - distance)
        heading = (heading + 180) % 360
        return pos, heading

    def sample_points(self, count=20):
        return [
            self._forward(self.length * i / count)[0] for i in range(count + 1)
        ]
