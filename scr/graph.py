"""TaxiWayの地図(静的な構造)の構築と、その描画だけを担当するモジュール。

ルート探索・選択肢の計算・移動の組み立てといった『移動・計画』に
関するロジックは movement.py 側にある。draw_graph は、計画中の経路を
プレビュー表示したい場合、呼び出し側(main.py)が
movement.build_preview_legs 等で組み立てた leg のリストを
planned_legs として渡すことで、それを青い線で重ねて描画する
(このモジュール自身はルートをlegに変換する処理を持たない)。
"""

import pygame

from settings import NODE_RADIUS, WIDTH, HEIGHT
from curve import Segment, CornerSegment
from maps import nodes, TAXIWAYS, CURVES, CORNER_CONNECTORS


def _build_links():
    result = {node_id: [] for node_id in nodes}
    for path in TAXIWAYS.values():
        for a, b in zip(path, path[1:]):
            result[a].append(b)
            result[b].append(a)
    return result


def _build_edge_taxiway():
    """辺(ノードのペア)がどのTaxiWayに属するかのマップ"""
    mapping = {}
    for taxiway_name, path in TAXIWAYS.items():
        for a, b in zip(path, path[1:]):
            mapping[frozenset((a, b))] = taxiway_name
    return mapping


def _build_edge_segments():
    """辺(ノードのペア)ごとの実際の形状(直線 or 曲線)を表す Segment"""
    segments = {}
    for path in TAXIWAYS.values():
        for a, b in zip(path, path[1:]):
            key = frozenset((a, b))
            if key in segments:
                continue
            control = CURVES.get(key)
            segments[key] = Segment(a, b, nodes[a], nodes[b], control)
    return segments


def _normalize_controls(alpha):
    """Segment が受け付ける制御点の指定(None / 1個のtuple / 複数のtupleの
    リスト)を、常に「制御点のリスト」の形に揃えて返す。"""
    if alpha is None:
        return []
    if isinstance(alpha, tuple) and len(alpha) == 2 and all(
        isinstance(v, (int, float)) for v in alpha
    ):
        return [alpha]
    return list(alpha)


def _build_corner_segments():
    """交差点ごとの手動フィレット(CORNER_CONNECTORS、X・Y・Z・t・s・alpha指定)
    から CornerSegment を作る。キーは (via_node=X, from_node=Y, to_node=Z)。
    Y-X区間・X-Z区間それぞれ実際に描画されている辺(edge_segments)を
    使って組み立てるので、直線のTaxiWayでも曲線のTaxiWayでも使える。

    Y->X->Z 方向を1つ定義すると、逆向き(Z->X->Y)のフィレットも
    自動的に生成する。t と s を入れ替え、フィレット部分のベジエ制御点の
    順序も反転させることで、見た目としては全く同じ曲線を逆向きに
    たどる形になる(数学的に、2次/3次ベジエは制御点の順序を反転させて
    始点・終点を入れ替えると同じ曲線を逆パラメータでたどることになる)。
    (X, Z, Y) が CORNER_CONNECTORS 側で明示的に定義されている場合は、
    そちらを優先し自動生成はスキップする。"""
    segments = {}
    for via_node, connectors in CORNER_CONNECTORS.items():
        for (y_node, z_node), (t, s) in connectors.items():
            alpha = nodes[via_node]
            edge_yx = get_segment(y_node, via_node)
            edge_xz = get_segment(via_node, z_node)

            # edge_yx/edge_xz はここでは常に地図上の生の辺(Segment)なので、
            # 両端のノードIDは必ず異なる。この時点でのID比較は安全であり、
            # 一度だけ行って以降はこのbool(yx_reversed/xz_reversed)を
            # 持ち回るだけにする(CornerSegment内部ではノードIDを一切
            # 見ない設計。curve.CornerSegmentのdocstring参照)。
            yx_reversed = edge_yx.node_a != y_node
            xz_reversed = edge_xz.node_a != via_node

            segments[(via_node, y_node, z_node)] = CornerSegment(
                edge_yx, edge_xz, yx_reversed, xz_reversed, t, s, alpha,
                node_a=y_node, node_b=z_node, node_x=via_node,
            )

            reverse_key = (z_node, y_node)
            if reverse_key not in connectors:
                reversed_alpha = list(reversed(_normalize_controls(alpha)))
                # 逆向き(Z->X->Y)は、edge_xzをZ->X方向に、edge_yxをX->Y方向に
                # 読む。それぞれ元の向き(X->Z / Y->X)の否定になる。
                segments[(via_node, z_node, y_node)] = CornerSegment(
                    edge_xz, edge_yx, not xz_reversed, not yx_reversed,
                    s, t, reversed_alpha,
                    node_a=z_node, node_b=y_node, node_x=via_node,
                )

    return segments


def _build_node_display_names():
    """ノードの表示名 = そこを通る全TaxiWay名を組み合わせたもの"""
    taxiways_at_node = {node_id: set() for node_id in nodes}
    for taxiway_name, path in TAXIWAYS.items():
        for node_id in path:
            taxiways_at_node[node_id].add(taxiway_name)

    return {
        node_id: "-".join(sorted(names))
        for node_id, names in taxiways_at_node.items()
    }


links = _build_links()
edge_taxiway = _build_edge_taxiway()
edge_segments = _build_edge_segments()


def get_segment(a, b):
    return edge_segments[frozenset((a, b))]


# CORNER_CONNECTORS は edge_segments (get_segment) に依存するので、
# それらが揃った後に組み立てる。
corner_segments = _build_corner_segments()
node_display_names = _build_node_display_names()


# --- 連続する2つの角(コーナー)が物理的に連結可能かどうかの判定 ---
#
# via1 で「y1 から来て via2 へ抜ける」フィレット(entry_corner)を使った
# 直後、via2 で「via1 から来て z2 へ抜ける」フィレット(exit_corner)を
# 選ぼうとした場合、この2つが同じ辺(via1-via2)上で物理的に両立するかを
# 判定する。
#
# entry_corner の s (via1から見た、via1-via2辺への吐き出し位置)より
# 手前の区間 [0, s] は、entryのフィレット自身が既に占有している。
# exit_corner の t (via2から見た、直進していなければならない距離)は、
# via1-via2辺上で [edge_length - t, edge_length] の区間が直進である
# ことを要求する。
#
# entry.s + exit.t <= edge_length であれば、2つのフィレットの占有領域は
# 重ならず(間に直進区間が残るだけ)物理的に両立する。
# entry.s + exit.t >  edge_length だと、2つのフィレットの領域が重なって
# しまい、実際にはあり得ない形状になるため、連結不可(そのルートは
# 選択肢に出してはいけない)と判定する。
_CHAIN_TOLERANCE = 1e-6


def corners_chain_compatible(entry_corner_key, exit_corner_key):
    """entry_corner_key = (via1, y1, via2) を実際に使って via2 へ到着した
    直後、続けて exit_corner_key = (via2, via1, z2) のフィレットへ入れるか
    どうかを、via1-via2間の辺の上での占有区間の重なりで判定する。
    どちらのキーも corner_segments に存在している前提。"""
    entry = corner_segments[entry_corner_key]
    exit_ = corner_segments[exit_corner_key]
    via1, via2 = entry_corner_key[0], entry_corner_key[2]
    edge_length = get_segment(via1, via2).length
    return entry.s + exit_.t <= edge_length + _CHAIN_TOLERANCE


def _build_edge_endpoint_trims():
    """CORNER_CONNECTORSで手動フィレットが定義されている交差点(via)ごとに、
    そのviaに接する辺の『via側の端』が、そのフィレットによってどれだけ
    直線区間から置き換えられているか(t または s の距離)を集計する。

    キーは (frozenset(辺), via側のノードID)。同じ辺の同じ端を複数の
    コーナー定義が使っている場合(例: BでA12からもHからも曲がれる等)は、
    最小値を採用する。最大値を採用してしまうと、その辺の端を使う
    フィレットの中で一番短いもの(t/sが小さいもの)が描画されないまま
    直線側だけが大きく削られてしまい、直線ともフィレットとも重ならない
    「隙間」が交差点付近にできてしまう。最小値にしておけば、直線は
    どのフィレットとも重ならない安全な範囲までしか削られず、
    隙間ではなく(見た目上ほぼ気にならない)重なりが生じるだけになる。

    これは静的なTaxiWay地図の描画専用の情報で、実際の移動には影響しない。
    CORNER_CONNECTORSの定義が無い辺・端は0のままなので、今まで通り
    直線で描画される。"""
    trims = {}
    for (via_node, y_node, z_node), corner in corner_segments.items():
        yx_key = frozenset((y_node, via_node))
        if (yx_key, via_node) in trims:
            trims[(yx_key, via_node)] = min(trims[(yx_key, via_node)], corner.t)
        else:
            trims[(yx_key, via_node)] = corner.t

        xz_key = frozenset((via_node, z_node))
        if (xz_key, via_node) in trims:
            trims[(xz_key, via_node)] = min(trims[(xz_key, via_node)], corner.s)
        else:
            trims[(xz_key, via_node)] = corner.s
    return trims


edge_endpoint_trims = _build_edge_endpoint_trims()


def node_display_name(node_id):
    return node_display_names.get(node_id, node_id)


# --- 描画 ---

def _draw_legs_path(screen, camera, legs, color, width):
    """legs(build_movement_legsで得られた(start_node, end_node, segment,
    reversed)
    のリスト)を、実際の形状(直線/曲線、交差点の手動フィレット含む)通りに
    描画する。ルート(ノード列)からlegを組み立てる処理自体は
    movement.build_movement_legs 側の仕事で、このモジュールは
    出来上がったlegを描くだけ。"""
    all_points = []
    for start_node, end_node, segment, seg_reversed in legs:
        raw_points = segment.sample_points(count=16)
        if seg_reversed:
            raw_points = list(reversed(raw_points))
        screen_points = [
            camera.world_to_screen(px, py, WIDTH, HEIGHT) for px, py in raw_points
        ]
        if all_points:
            screen_points = screen_points[1:]
        all_points.extend(screen_points)

    if len(all_points) >= 2:
        pygame.draw.lines(screen, color, False, all_points, width)


def _draw_edge(screen, camera, u, v, color, width):
    """静的なTaxiWay地図上の1本の辺(u-v)を描画する。

    CORNER_CONNECTORSでu側・v側のどちらか(または両方)がその交差点の
    フィレットに置き換えられている場合は、その分だけ直線の描画区間を
    短くする。これにより、従来のようにノードを直線がまたいで突き抜ける
    見た目にはならず、フィレットが定義されている交差点では丸めカーブ
    (_draw_corner_filletsが別途描画する)だけが見える。
    CORNER_CONNECTORSの定義が無い辺・端は今まで通りそのまま直線で
    描画される(trimが0のため)。"""
    segment = get_segment(u, v)
    key = frozenset((u, v))
    trim_u = edge_endpoint_trims.get((key, u), 0.0)
    trim_v = edge_endpoint_trims.get((key, v), 0.0)
    length = segment.length

    start_d = min(trim_u, length)
    end_d = max(length - trim_v, start_d)
    if end_d - start_d < 1e-6:
        # 辺の全体がフィレットに置き換えられていて、直線として残る部分が無い
        return

    # segment は get_segment で取得した生の辺(両端ノードIDは必ず異なる)
    # なので、ここでのID比較は安全。
    seg_reversed = segment.node_a != u
    count = 16
    raw_points = [
        segment.point_and_heading(
            start_d + (end_d - start_d) * i / count, reversed=seg_reversed
        )[0]
        for i in range(count + 1)
    ]
    screen_points = [
        camera.world_to_screen(px, py, WIDTH, HEIGHT) for px, py in raw_points
    ]
    if len(screen_points) >= 2:
        pygame.draw.lines(screen, color, False, screen_points, width)


def _draw_corner_fillets(screen, camera, width):
    """CORNER_CONNECTORS で定義された交差点フィレット部分
    (Segment.fillet、Y側t・Z側sの直線区間を除いた曲線本体)を、
    通常のTaxiWayの辺と同じ見た目で重ねて描画する。
    直線区間自体は draw_graph の通常ループで既に描画されているので、
    ここでは丸め部分だけを追加すればよい。"""
    for corner in corner_segments.values():
        raw_points = corner.fillet.sample_points(count=16)
        screen_points = [
            camera.world_to_screen(px, py, WIDTH, HEIGHT) for px, py in raw_points
        ]
        if len(screen_points) >= 2:
            pygame.draw.lines(screen, (0, 180, 0), False, screen_points, width)


def draw_graph(screen, font, camera, planned_legs=None):
    """地図全体(TaxiWayの辺・交差点フィレット)を描画する。

    planned_legs が渡された場合(movement.build_preview_legs等で
    組み立てたleg列)、その経路を青い線で重ねて描画する。これにより、
    movement.py側でどんな経路(交差点の複合カーブ込み)を計画していても、
    graph.py はそのlegをそのままなぞって描くだけでよい。"""
    drawn = set()
    line_width = max(1, round(2 * camera.zoom))
    for node_id, neighbors in links.items():
        for neighbor in neighbors:
            edge = tuple(sorted([node_id, neighbor]))
            if edge in drawn:
                continue
            drawn.add(edge)

            # 実際の形状(直線/曲線)通りに描画する。CORNER_CONNECTORSで
            # 手前・先がフィレットに置き換えられている交差点では、その分
            # だけ直線を短くする(ノードをまたいで突き抜ける直線は廃止し、
            # 丸めカーブ側に見た目を任せる)。
            _draw_edge(screen, camera, node_id, neighbor, (0, 180, 0), line_width)

            p1 = camera.world_to_screen(*nodes[node_id], WIDTH, HEIGHT)
            p2 = camera.world_to_screen(*nodes[neighbor], WIDTH, HEIGHT)
            edge_taxiway_name = edge_taxiway.get(frozenset(edge), "")
            label = pygame.font.SysFont(None, int(24 * camera.zoom)).render(edge_taxiway_name, True, (255, 255, 255))
            screen.blit(label, ((p1[0] + p2[0]) / 2, (p1[1] + p2[1]) / 2))

    # CORNER_CONNECTORSで定義した交差点の丸め(フィレット)部分を重ねて描画する。
    # 実際の移動もこのフィレットをなぞるので、見た目と挙動が一致する。
    _draw_corner_fillets(screen, camera, line_width)

    # for node_id, pos in nodes.items():
    #     sx, sy = camera.world_to_screen(pos[0], pos[1], WIDTH, HEIGHT)
    #     color = (0, 255, 0)
    #     radius = NODE_RADIUS * camera.zoom
    #     pygame.draw.circle(screen, color, (sx, sy), max(1, int(radius)))
    #     label = font.render(node_display_name(node_id), True, (255, 255, 255))
    #     screen.blit(label, (sx - 5 * camera.zoom, sy - 25 * camera.zoom))

    # 計画中(キューにためた)経路を、実際の形状(曲線含む)通りに重ねて表示する
    if planned_legs:
        plan_width = max(2, round(3 * camera.zoom))
        _draw_legs_path(screen, camera, planned_legs, (0, 160, 255), plan_width)