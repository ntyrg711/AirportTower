"""機体の『移動・経路計画』に関するロジックをまとめたモジュール。

graph.py が地図の静的な構造(ノード・辺・フィレットの形状)と、その
描画だけを担当するのに対し、movement.py は
  - あるノードから、実際にどこへ進めるか(選択肢)を求める
  - 選んだルート(ノードの並び)を、実際に移動する単位である
    『leg』(直線/曲線の1区間、または交差点をまたいだ複合カーブ)
    のリストに変換する
  - 機体ごとの『計画(キュー)』の状態を持ち、選択の積み増し・確定・
    破棄を扱う(FlightPlan)
  - 確定した計画を、実際に移動を開始できる形(legs等)に組み立てる
    (resolve_route)
  - UI上の選択肢ボタンの並び(get_destination_buttons)

を担当する。aircraft.py はこのモジュールに計画を求め、返ってきた
legs・向きに従って実際の移動(フレームごとの座標更新)と描画だけを行う。

【停止(hold short)について】
機体が「次の指示待ち」で完全に止まるとき(legsが空になるとき)は、
常に交差点の手前で止まる。停止位置は、その交差点・その進入方向に
定義されているCORNER_CONNECTORSのフィレットのうち最も遠い
(=ノードから最も離れた)t/sの点から、さらにSAFE_DISTANCEだけ手前
になる(交差点ごと・進入方向ごとにただ1つの位置)。ノードの座標に
ピッタリ乗って止まることはない。
これは直線区間だけでなく、交差点をまたいだ複合カーブ(CornerSegment)
の途中でも起こり得る。その場合、止まった原因のleg(start_node,
end_node, segment)自体と、その中での進捗(distance)を
Aircraft側が覚えておく(approach_leg / approach_progress)。
次の指示が来たとき、bridge_from_approachはノードペアから辺を
作り直すのではなく、そのleg自体を新しい経路の先頭にそのまま
継ぎ足すことで、直線でも曲線でもワープせずに滑らかに続きを進める。

【移動中の再計画について】
機体が完全に停止していなくても(まだ現在のlegを進行中でも)、次の
指示の組み立てを始めることができる。計画を組み立て始めた瞬間
(Aircraft._begin_replan)に、その時点で進行中のleg1つだけを残して
それより先の(古い経路の)legsを打ち切り、そのlegの終端で自然に
hold shortするよう停止距離を計算し直す。これにより、以降は通常の
hold short(legsが空になり、approach_leg/approach_progressを使って
続きを繋ぐ)の仕組みがそのまま使える。_effective_planning_stateは、
停止中・移動中のどちらでも、次の指示の選択肢を求めるための基準
(ノード・向き・除外方向・残り距離)を統一的に返す。
"""

import math
from tracemalloc import start

import pygame

from settings import SAFE_DISTANCE
from curve import CornerSegment
from graph import (
    links,
    edge_taxiway,
    edge_segments,
    get_segment,
    corner_segments,
    corners_chain_compatible,
)


# --- ルート(ノード列) -> leg(移動単位)への変換 ---

def _corner_params(via, prev_node, next_node):
    """既に組み立て済みのcorner_segmentsから、その角のパラメータ
    (t, s, alpha=フィレットの制御点)を取り出す。alphaは既に絶対座標に
    正規化済みの制御点リストなので、そのまま新しいCornerSegmentの
    controlsとして再利用できる。"""
    seg = corner_segments[(via, prev_node, next_node)]
    return seg.t, seg.s, seg.fillet.controls


def _edge_reversed(edge, from_node):
    """edge(get_segmentで取得した生の辺。両端ノードIDは必ず異なる)を
    from_node起点で読むために、point_and_heading(distance, reversed=...)
    へ渡すべきreversedフラグを返す。

    ここでのID比較は、対象が常に地図上の生の1区間(Segment、両端が
    別ノード)である場合に限って呼び出すこと。チェーンされた
    CornerSegment(内部で同じノードIDを繰り返し使っているかもしれない
    オブジェクト)に対しては絶対に使わない。この関数はその境界を
    はっきりさせるためにあえて単独の関数として切り出している。"""
    return edge.node_a != from_node


def _build_chained_corner(chain_nodes, incoming_edge=None, incoming_reversed=None):
    """chain_nodes = [start, via1, via2, ..., viaM, end] を、途中で一切
    止まらない1本の複合カーブに組み立てる。戻り値は、
    「chain_nodes[0]を起点に、reversed=Falseでchain_nodes[-1]方向へ
    進める」というCornerSegmentオブジェクト。

    via1〜viaMはすべて、隣り合うノードの並びからCORNER_CONNECTORSの
    角として確認済み(build_movement_legs側で判定済み)という前提。

    ポイントは、CornerSegmentがSegmentと全く同じインターフェース
    (length/point_and_heading/sample_points)を持つこと。これを利用して、
    1つ目の角を組み立てた結果(CornerSegment)を、2つ目の角の
    『入口側の辺』としてそのまま渡す。これにより、
    - 1つ目の角は「startからvia1の先、via1-via2辺の途中まで」を
      正しく1本のカーブとして表現し、
    - 2つ目の角を作るときは、via1-via2辺を新たに独立して使うのではなく
      1つ目の角(そこまでの経路全体)を経由地点として扱うため、
      via1-via2間の辺を二重に消費することなく、via2の手前で
      自然に折れ曲がりを継ぎ足せる。

    こうして角がいくつ連続していても、同じ物理区間を重複して使わずに
    1本の連続した滑らかな経路として繋げられる。

    【進行方向をノードIDで判定しないことについて】
    以前の実装は、チェーン全体で固定の「アンカーノードID」を
    CornerSegment.node_aとして持ち回し、進行方向の判定(from_node ==
    node_a/node_b)をID一致で行っていた。これは、ルートが同じノードを
    2回以上通る(ループする)と、チェーンの途中に同じIDが再登場して
    判定が衝突し、進行方向を取り違えるという欠陥を持っていた
    (過去のバグ)。

    この実装ではノードIDによる判定を一切行わない。各ステップで
    「edge_yxをY->X方向に読むにはreversedをどうすればよいか」を、
    その場で新規に取得した生の辺(get_segment、両端が必ず別ノードで
    ID比較が安全)、または直前に自分自身が組み立てたCornerSegment
    (自分で組み立てた以上、その向きは常にわかっている)のどちらかから
    構造的に確定させ、boolとして引き回す。ループするルートで同じ
    ノードIDが何度登場しても、この判定は一切ノードIDを参照しないため
    衝突しようがない。

    incoming_edge / incoming_reversed を渡すと、chain_nodes[0]から
    chain_nodes[1]へ向かう区間を通常のget_segmentで作り直すのではなく、
    指定した既存の区間(Segmentでも、hold short中のapproach_legのように
    CornerSegmentが連なった複合カーブでもよい)を1つ目の角の『入口側の
    辺』としてそのまま使う。incoming_reversedは、その既存区間を
    chain_nodes[0]起点で読むためのreversedフラグで、呼び出し側
    (build_movement_legs)がその区間を作った/受け取った時点で確定させて
    渡す。これにより、既に曲がりながら進入してきた経路の続きとして、
    ワープや直線化を挟まずに次の角へなめらかに繋げられる
    (bridge_from_approach参照)。"""
    if incoming_edge is None:
        incoming_edge = get_segment(chain_nodes[0], chain_nodes[1])
        incoming_reversed = _edge_reversed(incoming_edge, chain_nodes[0])

    for k in range(1, len(chain_nodes) - 1):
        via = chain_nodes[k]
        next_node = chain_nodes[k + 1]
        prev_node = chain_nodes[k - 1]
        t, s, alpha = _corner_params(via, prev_node, next_node)

        outgoing_edge = get_segment(via, next_node)
        outgoing_reversed = _edge_reversed(outgoing_edge, via)

        incoming_edge = CornerSegment(
            incoming_edge, outgoing_edge, incoming_reversed, outgoing_reversed,
            t, s, alpha,
            node_a=chain_nodes[0], node_b=next_node, node_x=via,
        )
        # 自分でここで組み立てたCornerSegmentは、常に
        # chain_nodes[0]->next_node がreversed=Falseの向きになるよう
        # 構築しているので、次の反復の「入口側の辺」として使うときも
        # reversedは常にFalseでよい(IDの再照合は不要)。
        incoming_reversed = False

    return incoming_edge


def build_movement_legs(route, entry_leg=None, entry_progress=0.0):
    """route(ノードIDの並び)を、実際に移動する単位である『leg』の
    リストに変換する。
    普通の1区間はそのまま1leg。CORNER_CONNECTORS に手動で登録されている
    (via, from, to)の組み合わせは、viaのノードで止まらず、1本の
    なめらかな曲線としてまとめて進む。

    交差点が連続していて、両方ともCORNER_CONNECTORSの角になっている
    場合(例: A-B-C-DでBもCも曲がる指定がある)は、経路全体を
    1ノードずつ先読みして『どこまで連続して曲がれるか』を判定し、
    その区間をまとめて1本の複合カーブ(_build_chained_corner)にする。
    ルートが同じノードを2回以上通る(ループする)場合でも、
    _build_chained_cornerの向き判定はノードIDに依存しないため、
    連続して曲がれる区間はそのままの長さで複合カーブにまとめられる
    (以前あった「アンカー再登場の手前で機械的に打ち切る」処理は、
    その必要が無くなったため廃止した)。

    entry_leg: route[0]へ実際に進入してきた直前のleg
    (start_node, end_node, segment, reversed)。bridge_from_approachから、
    hold short中(またはまだ移動中)のapproach_legを渡すために使う。
    entry_progress: entry_leg内で、そのstart_nodeから実際に進んだ距離
    (approach_progress)。

    【entry_legの扱いについて】
    entry_legのsegmentは、単純な1区間ならroute[0]の直前ノードとの
    間の生の辺そのものだが、過去に何度も複合カーブ(CornerSegment)を
    継ぎ足してきたentry_legの場合、そのsegmentは「継ぎ足しが始まった、
    はるか昔の起点」からの弧長座標のままで、以後ずっと更新されない。
    これをそのまま次の複合カーブの入口辺として使い続けると、legの
    長さが無限に伸び、描画が毎回はるか昔の起点から辿り直される。

    そこで、entry_legの「今いる場所」が、直前ノード
    (leg_entry_node(entry_leg))とroute[0]を結ぶ実際の地図の辺
    (get_segment)の中に収まっているかどうかを確認する。
    entry_legがCornerSegmentのチェーンであっても、その末尾(leg3)は
    必ずget_segment(via, next_node)そのものなので、entry_progressが
    実際にそのleg3(フィレットを通過し終えた後の生の辺の部分)まで
    進んでいれば、幾何学的に完全に同一であり、entry_leg自体(はるか
    昔の履歴込み)を捨てて、直前ノードを新たな起点とする『素の』route
    (通常の先読み・結合ロジック)にそのまま合流させてよい。

    重要なのは、この置き換えが安全なのは「距離(remaining)が収まって
    いるかどうか」だけでなく、実際に entry_progress がleg3(生の辺の
    部分)に到達しているかどうかも必要な点。entry_legがまだ
    leg1(直進区間)やfillet(曲線区間)の途中にある場合、そこはlocal_edge
    (直前ノード-route[0]間の生の辺)とは全く別の形状なので、たとえ
    残り距離だけが収まって見えても置き換えてはいけない(置き換えると、
    実際のカーブ上の位置とは無関係な、local_edge上のどこかへ座標が
    ワープしてしまう)。この場合(まだ本当にentry_leg全体の途中、
    つまり古い区間を移動中)は、従来通りentry_leg自体を先頭に維持し、
    warpを避ける。

    各legは (start_node, end_node, segment, reversed) のタプル。
    reversedは、segment.point_and_heading(distance, reversed=reversed)を
    呼ぶとstart_nodeを起点にした位置が返ってくることを表すフラグで、
    ここで一度確定させたら、以降このlegを使う側は一切ノードIDを
    見ずにこのフラグだけを使い回す。
    戻り値は (legs, start_progress) のタプル。start_progressは、
    返されたlegs[0]のstart_nodeから実際に進んだ距離。"""
    legs = []
    i = 0
    start_progress = entry_progress

    if entry_leg is not None and len(route) >= 2:
        entry_node = leg_entry_node(entry_leg)
        local_edge = get_segment(entry_node, route[0])
        remaining = entry_leg[2].length - entry_progress

        # entry_legが交差点のフィレット(CornerSegment)の場合、その末尾
        # (leg3)は必ずlocal_edgeそのものと幾何学的に一致するが、
        # leg1(直進区間)やfillet(曲線区間)はlocal_edgeとは全く別の形状
        # なので、そこに位置している間は「距離が収まっているかどうか」
        # だけでは同一視できない。entry_progressが実際にleg3(フィレット
        # を通過し終えた後の生の辺の部分)まで進んでいるかを、形状ごと
        # 確認してから判定する。
        entry_segment = entry_leg[2]
        if hasattr(entry_segment, "leg1_length"):
            on_raw_tail = entry_progress > entry_segment.leg1_length + entry_segment.fillet.length + 1e-9
        else:
            # entry_leg自体がもともと生のSegment(角なし)なので、local_edge
            # と同一の辺の上にいる。
            on_raw_tail = True

        if on_raw_tail and remaining <= local_edge.length + 1e-6:
            # 今いる場所はentry_node-route[0]間のローカルな生の辺の中に
            # 収まっている(かつ実際にその辺の形状の上にいる)。entry_leg
            # (はるか昔の履歴)を捨てて、直前ノードを先頭に継ぎ足した
            # 『素のroute』として以降の通常ロジックにそのまま合流させる。
            entry_leg = None
            start_progress = local_edge.length - remaining
            route = [entry_node] + list(route)
        else:
            # まだentry_leg全体の途中(本当にmid-flight中)。
            # entry_leg自体を先頭に維持し、warpを避ける。
            extended = [entry_node] + list(route)
            run_end = 0  # extended上のインデックスで、最後に確認できたvia
            j = 0
            while (
                j + 2 < len(extended)
                and (extended[j + 1], extended[j], extended[j + 2]) in corner_segments
            ):
                run_end = j + 1
                j += 1

            if run_end > 0:
                chain_nodes = extended[: run_end + 2]
                segment = _build_chained_corner(
                    chain_nodes, incoming_edge=entry_leg[2], incoming_reversed=entry_leg[3]
                )
                legs.append((entry_leg[0], chain_nodes[-1], segment, False))
                # extended上のインデックス(len-1) -> route上のインデックスへ変換
                # (extended[0]がentry_nodeの分だけ1つずれている)
                i = len(chain_nodes) - 2
            else:
                # route[0]では曲がらない(または角の定義が無い)ので、
                # entry_leg自体をそのまま最初のlegとして使う。
                legs.append(entry_leg)

    while i < len(route) - 1:
        # route[i] を起点に、隣り合うノード3つ組が連続してCORNER_CONNECTORS
        # に登録されている限り、via候補を1つずつ先読みする。
        run_end = i  # 最後に確認できたviaのインデックス(route上の位置)
        j = i
        while j + 2 < len(route) and (route[j + 1], route[j], route[j + 2]) in corner_segments:
            run_end = j + 1
            j += 1

        if run_end > i:
            chain_nodes = route[i:run_end + 2]
            segment = _build_chained_corner(chain_nodes)
            legs.append((chain_nodes[0], chain_nodes[-1], segment, False))
            i += len(chain_nodes) - 1
            continue

        a, b = route[i], route[i + 1]
        edge = get_segment(a, b)
        legs.append((a, b, edge, _edge_reversed(edge, a)))
        i += 1
    return legs, start_progress


# --- 現在地からの選択肢の探索 ---

def angle_difference(a, b):
    """2つの角度(度)の差を 0〜180 の範囲で返す"""
    diff = (a - b) % 360
    if diff > 180:
        diff = 360 - diff
    return diff


def _is_turn_allowed(
    via_node,
    from_node,
    to_node,
    from_heading,
    approach_remaining=math.inf,
    entry_corner=None,
):
    """via_nodeで、from_nodeから来てto_nodeへ向かう移動が許可されるかどうか。

    graph.pyが実際に描画する『物理的につながった経路』(=同じTaxiWayの
    直進、またはCORNER_CONNECTORSに定義されたフィレット)だけを選択肢に
    出す。角度が浅い/急といった曖昧な基準(旧140°ルール)では判定しない。
    そのため、グラフ上ではノードが辺で繋がっていても、直進でもフィレット
    でもない組み合わせは選択肢に出せないことがある(その場合はmaps.pyの
    CORNER_CONNECTORSに該当する角を追加する必要がある)。

    - from_node が None の場合(まだ何のTaxiWayにも乗っていない、
      出発直後の最初の一歩): 参照する『今のTaxiWay』が無いので、
      向いている方向(from_heading)を基準にした140°ルール(真後ろ
      禁止)だけで判定する。これは曲線の要不要の話ではなく、機体が
      物理的に不可能な急な方向転換をいきなり選べないようにするための
      制約。
    - from_node がある場合:
        - from_node->via_node と via_node->to_node が同じTaxiWayの
          区間なら、そのまま直進するだけなのでカーブ定義が無くても許可。
        - それ以外(別のTaxiWayへ実際に曲がる)は、CORNER_CONNECTORSに
          (via_node, from_node, to_node)のフィレットが定義されていて、
          かつまだそのフィレットの手前(approach_remaining >= t)に
          いる場合だけ許可する。定義が無ければ許可しない。
        - さらに、entry_corner(from_nodeへ到着する直前に実際に使った
          フィレットのcorner_segmentsキー。直進で到着した場合や
          出発直後はNone)が渡されている場合、そのフィレットと
          今回のフィレット(via_node, from_node, to_node)が
          from_node-via_node間の辺の上で物理的に連結可能かどうか
          (graph.corners_chain_compatible)も確認する。連結不可能
          (2つのフィレットの占有区間が重なってしまう)場合は、
          たとえ単独では定義されているフィレットでも許可しない。
    """
    if from_node is None:
        segment = get_segment(via_node, to_node)
        _, direction = segment.point_and_heading(0.0, reversed=_edge_reversed(segment, via_node))
        return angle_difference(direction, from_heading) < 140

    edge_in = edge_taxiway.get(frozenset((from_node, via_node)))
    edge_out = edge_taxiway.get(frozenset((via_node, to_node)))
    if edge_in is not None and edge_in == edge_out:
        return True

    corner = corner_segments.get((via_node, from_node, to_node))
    if corner is not None and approach_remaining >= corner.t:
        if entry_corner is not None and not corners_chain_compatible(
            entry_corner, (via_node, from_node, to_node)
        ):
            return False  # 直前に使ったフィレットと物理的に繋がらない
        return True
    return False


def _incoming_corner(aircraft, path, from_node, via_node):
    """from_node -> via_node への移動が、CORNER_CONNECTORSのフィレットを
    使ったものだった場合、そのcorner_segmentsキー
    (from_node, predecessor, via_node) を返す。

    predecessor(from_nodeに至る直前のノード)は、pathの中で最後に
    from_nodeが登場する位置の1つ前を使う(hold short直後の
    再計画などで同じノードが繰り返し登場していても、直近の経路を
    正しく参照するため、末尾から探す)。pathの先頭がfrom_nodeの場合は
    _effective_planning_state(aircraft)が返すexclude_prev(来た方向の
    ノード。完全に停止中ならapproach_from、まだ現在のlegを移動中なら
    そのlegの起点)を使う。

    直進(同一TaxiWay)で到着していた場合や、predecessorが無い
    (出発直後で、まだ何のフィレットも使っていない)場合はNoneを返す。

    これは、via_nodeでさらに別方向へ曲がろうとしたとき、その候補の
    フィレットが、from_nodeへの到着で既に使ったフィレットと物理的に
    連結可能か(graph.corners_chain_compatible)を調べるための
    entry_cornerを求めるために使う。"""
    idx = None
    for i in range(len(path) - 1, -1, -1):
        if path[i] == from_node:
            idx = i
            break
    if idx is None:
        return None

    predecessor = path[idx - 1] if idx > 0 else _effective_planning_state(aircraft)[2]
    if predecessor is None:
        return None

    edge_in = edge_taxiway.get(frozenset((predecessor, from_node)))
    edge_out = edge_taxiway.get(frozenset((from_node, via_node)))
    if edge_in is not None and edge_in == edge_out:
        return None  # 直進で到着しているので、連結すべきフィレットは無い

    key = (from_node, predecessor, via_node)
    return key if key in corner_segments else None


def get_route_to_next_junction(start_node, first_neighbor):
    """start_node から first_neighbor 方向へ、同じTaxiWayをたどるが、
    途中で別のTaxiWayと交差するノードに出会ったらそこで止める
    (その先には進まない)。そのノードが本当の終点(行き止まり)なら
    そのまま最後まで進む。(taxiway_name, route) のタプルを返す。"""
    taxiway_name = edge_taxiway[frozenset((start_node, first_neighbor))]
    route = [start_node, first_neighbor]
    prev_node, current_node = start_node, first_neighbor

    while True:
        other_neighbors = [n for n in links[current_node] if n != prev_node]

        continuation = None
        for n in other_neighbors:
            if edge_taxiway.get(frozenset((current_node, n))) == taxiway_name:
                continuation = n
                break

        has_branch = any(n != continuation for n in other_neighbors)
        if has_branch:
            break  # 別のTaxiWayとの交差点なので、ここで止める
        if continuation is None:
            break  # 本当の行き止まり(終点)

        route.append(continuation)
        prev_node, current_node = current_node, continuation

    return taxiway_name, route


def get_full_ride(start_node, first_neighbor):
    """start_node から first_neighbor 方向へ、同じTaxiWayに乗って
    本当の終点(行き止まり)まで一気に進んだルートを返す。
    get_route_to_next_junction と違い、途中で別のTaxiWayと交差する
    ノードがあっても止まらず、そのまま最後まで進む。
    (taxiway_name, route) のタプルを返す。"""
    taxiway_name = edge_taxiway[frozenset((start_node, first_neighbor))]
    route = [start_node, first_neighbor]
    prev_node, current_node = start_node, first_neighbor

    while True:
        next_node = None
        for n in links[current_node]:
            if n == prev_node:
                continue
            if edge_taxiway.get(frozenset((current_node, n))) == taxiway_name:
                next_node = n
                break
        if next_node is None:
            break
        route.append(next_node)
        prev_node, current_node = current_node, next_node

    return taxiway_name, route


def get_turn_options_along_ride(route, entry_corner=None):
    """route(あるTaxiWayに端から端まで乗った経路)の途中の交差点、
    および終点(route[-1])それぞれについて、そこで別のTaxiWayへ
    曲がれる(CORNER_CONNECTORSが定義されている)選択肢を集めて返す。

    entry_corner: route[0](このrideの出発点)へ到着する際に実際に
    使ったフィレットのcorner_segmentsキー(直進で到着していた場合や
    出発直後はNone)。route[0]の直後(route[1]でのvia判定)でだけ、
    そこでの候補フィレットがentry_cornerと物理的に連結可能かを
    確認する(_incoming_corner参照)。それより先のviaは、route内は
    常に同一TaxiWayの直進で結ばれているため、この確認は不要。

    終点(route[-1])も対象に含めるのは、Mのように短いTaxiWayが
    「自分自身はそこで行き止まりだが、別のTaxiWay(例: B)との
    交差点そのものが終点になっている」場合、そこがまさに
    「Yに乗っていてXとの交差点手前で止まる(hold short of X)」の
    本来の状況にあたるため。route[0](出発点)だけは、そこから見て
    「今乗っているTaxiWay」が定義できないので対象外のままにする。

    各要素は (taxiway_name, turn_route, hold_short_route) のタプル:
      taxiway_name    : 曲がった先のTaxiWayの名前(例: "X")
      turn_route      : 実際に曲がって、そのTaxiWayの本当の終点まで進む
                        経路(route[0] は曲がる交差点のノード)。
      hold_short_route: 「hold short of X」を選んだ場合の経路。
                        まだ曲がらず、今乗っている route(引数)の上を、
                        その交差点の手前(via)まで進むだけの経路。"""
    options = []
    for i in range(1, len(route)):
        via = route[i]
        prev_node = route[i - 1]
        continuation = route[i + 1] if i + 1 < len(route) else None

        # このノードに至る直前の区間の、到着した瞬間の向き(接線方向)
        prev_segment = get_segment(prev_node, via)
        _, arrival_heading = prev_segment.point_and_heading(
            prev_segment.length, reversed=_edge_reversed(prev_segment, prev_node)
        )

        # 今乗っているTaxiWay上で、まだ曲がらずviaの手前まで進む経路
        hold_short_route = route[: i + 1]

        # i==1(route[1]でのvia判定)のときだけ、entry_cornerとの連結
        # チェック対象にする。それ以降のviaはroute内の直進で繋がって
        # いるので対象外(entry_cornerを渡さない=Noneのまま)。
        current_entry_corner = entry_corner if i == 1 else None

        for neighbor in links[via]:
            if neighbor in (prev_node, continuation):
                continue
            if not _is_turn_allowed(
                via, prev_node, neighbor, arrival_heading, entry_corner=current_entry_corner
            ):
                continue
            new_taxiway_name, turn_route = get_full_ride(via, neighbor)
            options.append((new_taxiway_name, turn_route, hold_short_route))

    return options


def get_reachable_routes_from(
    node_id, heading, exclude_prev=None, approach_remaining=math.inf, entry_corner=None
):
    """node_id にいて heading の向きを向いている状態を基準に、選択可能な
    ルートの一覧を返す。各要素は (taxiway_name, route)。
    どの選択肢も、選んだTaxiWayに乗って本当の終点(行き止まり)まで
    一気に進む(get_full_ride)。途中の交差点で曲がれる場所は、
    ここでは出さず、その先を選んだ後の画面(get_reachable_routes参照)で
    改めて選択肢として出す。
    _is_turn_allowed により、直進(同じTaxiWay継続)か、フィレットが
    定義されている方向転換だけが選択肢に残る。

    approach_remaining: hold short中(交差点ノードの手前で待機中)の
    機体について、実際の物理的な現在地からnode_idまでの残り距離。
    通常(待機中でない)場合はmath.infのままでよく、この値は無視される。

    entry_corner: exclude_prev(直前のノード)からnode_idへの到着で
    実際に使ったフィレットのcorner_segmentsキー(直進で到着していた
    場合や出発直後はNone)。node_idでの候補フィレットがこれと物理的に
    連結可能かどうかも_is_turn_allowedが確認する(_incoming_corner参照)。"""
    routes = []

    for neighbor in links[node_id]:
        if neighbor == exclude_prev:
            continue

        if not _is_turn_allowed(
            node_id, exclude_prev, neighbor, heading, approach_remaining, entry_corner=entry_corner
        ):
            continue

        routes.append(get_full_ride(node_id, neighbor))

    return routes


def leg_entry_node(leg):
    """leg (start_node, end_node, segment, reversed) について、その終端
    (end_node)へ実際に進入してくる直前のノードを返す。

    普通の1区間(単純なSegment)ならstart_nodeそのものでよいが、
    交差点をまたいだ複合カーブ(CornerSegment、複数のフィレットを
    _build_chained_cornerで繋いだもの)の場合、start_nodeはチェーン
    全体の起点(遠く離れたノード)であり、end_node直前のノードでは
    ない。CornerSegmentは直近に通過したvia(フィレットの中心ノード)
    をnode_xとして持っている(チェーンしても、_build_chained_corner
    は毎回そのステップのvia・next_nodeでCornerSegmentを組み立て直す
    ため、最終的なnode_xは常に「最後に曲がった角」になる)ので、
    それがあればそちらを使う。node_xはラベルとしてのみ保持されて
    おり(curve.CornerSegment参照)、ここでの参照は表示・経路計画用の
    情報取得であって、進行方向の計算には使わない。"""
    start_node, _end_node, segment, _reversed = leg
    return getattr(segment, "node_x", start_node)


def locate_current_position(leg, progress):
    """leg (start_node, end_node, segment, reversed) を progress(そのlegの
    start_node起点の弧長)だけ進んだ、実際の物理的な『今いる場所』を、
    hold short中の機体が持つのと同じ形(current_node, approach_leg,
    approach_progress)に変換して返す。あわせて、その位置での向き
    (heading)と、直前に実際に使ったフィレットのcorner_segmentsキー
    (entry_corner。無ければNone)も返す。

    legが交差点をまたいだ複合カーブ(CornerSegmentのチェーン)の場合、
    外側(routeの終点に近い側)から内側(起点に近い側)へ辿り、
    まだそのフィレットの手前(=そのフィレットの向きがまだ確定していない)
    である間はさらに内側へ潜っていく。あるフィレットの中、またはその先
    (もうそのフィレットは通過確定済み)まで進んでいたら、そこで停止し、
    そのフィレットの出口ノード(node_b)を『まだ選び直せる直近のノード』
    として返す。どの階層でもフィレットに一切入っていなければ(生の
    Segmentまで潜り切ったら)、その生の辺の進行方向側のノードを返す。

    戻り値の(current_node, approach_leg, approach_progress)は、実際に
    その位置で機体が停止した場合の状態と完全に同じ形なので、
    get_reachable_routes_from・bridge_from_approach・resolve_route等、
    hold short時に使っている既存の仕組みへそのまま渡せる。これにより、
    『移動中の任意の時点を、hold short時と同じ状態表現に変換する』
    という変換として、Revise以外の将来の機能(その場で停止して
    ルートを再検索する、等)にもそのまま使い回せる。"""
    _start_node, _end_node, top_segment, top_reversed = leg
    _, heading = top_segment.point_and_heading(progress, reversed=top_reversed)

    def descend(segment, distance, reversed_flag):
        via = getattr(segment, "node_x", None)
        if via is None:
            # 生のSegment。まだどのフィレットにも入っていない。
            if not reversed_flag:
                near_node, far_node = segment.node_a, segment.node_b
            else:
                near_node, far_node = segment.node_b, segment.node_a
            approach_leg = (near_node, far_node, segment, reversed_flag)
            return far_node, approach_leg, distance, None

        # CornerSegmentの内部は常にY->X->Z(edge_yx->fillet->edge_xz)の
        # 向きで構築されているので、distance/reversed_flagを、この
        # セグメント自身の内部の弧長(Yを起点にした順方向)へ変換する。
        x = distance if not reversed_flag else segment.length - distance

        if x <= segment.leg1_length:
            # まだこの角(via)のフィレットの手前。もっと内側(より早い
            # 角、または生の辺)へ潜って続きを探す。
            return descend(segment.edge_yx, x, segment.yx_reversed)

        # この角(via)のフィレットの中、またはその先まで進んでいる
        # =この角は既にコミット済み。次に選び直せるのはこの角の
        # 出口ノード(node_b)。
        edge_yx = segment.edge_yx
        y1 = getattr(edge_yx, "node_x", None)
        if y1 is None:
            y1 = edge_yx.node_a if edge_yx.node_a != via else edge_yx.node_b

        approach_leg = (y1, segment.node_b, segment, False)
        entry_key = (via, y1, segment.node_b)
        entry_corner = entry_key if entry_key in corner_segments else None
        return segment.node_b, approach_leg, x, entry_corner

    current_node, approach_leg, approach_progress, entry_corner = descend(
        top_segment, progress, top_reversed
    )
    return current_node, approach_leg, approach_progress, heading, entry_corner


def _effective_planning_state(aircraft):
    """次の指示の選択肢を組み立てるための基準
    (node, heading, exclude_prev, approach_remaining)を、
    機体が停止中か移動中かに関わらず統一的に返す。

    - 移動中(aircraft.legsが空でない): 現在進行中のleg
      (aircraft.legs[0])を最後まで終えたと仮定した場合に到達する
      ノード・向きを基準にする。approach_remainingには、そのlegの
      残り距離(まだ進んでいない分)を渡すことで、まだ十分な距離が
      残っていないフィレットへの進入を正しく弾く。
    - 停止中(aircraft.legsが空): 従来通りaircraft.current_node /
      aircraft.heading / aircraft.approach_from /
      aircraft.approach_remainingをそのまま使う。"""
    if aircraft.legs:
        start_node, end_node, segment, seg_reversed = aircraft.legs[0]
        remaining = segment.length - aircraft.leg_progress
        _, heading = segment.point_and_heading(segment.length, reversed=seg_reversed)
        exclude_prev = leg_entry_node(aircraft.legs[0])
        return end_node, heading, exclude_prev, remaining

    return (
        aircraft.current_node,
        aircraft.heading,
        aircraft.approach_from,
        aircraft.approach_remaining,
    )


def get_reachable_routes(aircraft):
    """選択中の機体が今選べるルートの一覧を返す。
    各要素は (taxiway_name, route, hold_short_route) のタプル。

    機体が完全に停止しているか、まだ現在のlegを移動中かに関わらず
    呼び出せる。移動中の場合の基準(どのノードから選べるか)は
    _effective_planning_stateが返す、現在進行中のlegを終えた先の
    ノードになる(Aircraft._begin_replanが、実際にそこで一旦
    区切られるよう先のlegsを打ち切る)。

    hold_short_route は「hold short of {taxiway_name}」を選んだ場合の経路。
    「Yに乗っていて、この先のX方面の交差点の手前で止まる」という
    本来の意味に合わせて、これは曲がる先の選択肢(下記2)にだけ付く。
    今の位置・末端から新しく選べるTaxiWay(下記1、まだ何にも乗っていない
    状態で選ぶ最初のTaxiWayを含む)には、そもそも「今乗っている
    TaxiWay」が無い(またはまだ決まっていない)ので、hold shortの
    対象にならず、hold_short_routeはNoneになる。

    何も計画していない(aircraft.plan.pathがNone)場合は、現在地から
    選べるTaxiWayを、それぞれ本当の終点まで一気に進むルートとして出す
    (get_reachable_routes_from)。

    すでに何か計画済み(aircraft.plan.pathがある)場合は、次の2種類を
    合わせて出す:
      1. 今の計画の末端ノードから、改めて選べるTaxiWay
         (末端ノードがさらに別の交差点でもある場合。
         get_reachable_routes_fromと同じ)。hold_short_route は None。
      2. 直前に選んだ(積み増した)ライドの途中の交差点で、
         別のTaxiWayに曲がれる選択肢(get_turn_options_along_ride)。
         これを選ぶと、計画はその交差点まで切り詰められた上で
         新しいTaxiWayの終点まで繋げられ、次の画面でまた同じように
         「その新しいライドの途中で曲がれる場所」が出てくる
         (これが繰り返されることで、何回でも好きな場所で曲がれる)。
         hold_short_route には、まだ曲がらず今のTaxiWay上でその
         交差点の手前まで進む経路が入る。

    hold short直後(何も計画していない状態)で、まだノードの手前
    (SAFE_DISTANCEまたはフィレットの手前)に物理的に留まっている
    場合や、まだ現在のlegを移動中の場合は、_effective_planning_state
    が返すexclude_prev/approach_remainingを使って、フィレットがまだ
    使えるかどうかを正しく判定する。計画を積み増している最中
    (plan.path内)のノードは実際にはまだ物理的な現在地ではない
    (将来の仮想的な位置)ため、そちらはapproach_remainingの制限を
    受けない(math.inf)。"""
    if aircraft is None:
        return []

    plan = aircraft.plan
    if plan.path:
        tail = plan.path[-1]
        heading = plan.heading
        prev_node = plan.path[-2] if len(plan.path) > 1 else None
        if prev_node is None and plan.last_ride_route and len(plan.last_ride_route) > 1 and plan.last_ride_route[-1] == tail:
            # plan.pathがまだ1要素(=第1画面で「今乗っているTaxiWay自体」
            # を選んだ直後で、まだそこから何も曲がっていない状態)の場合、
            # 「来た方向」の情報はself.pathには乗せられない(乗せると、
            # confirm時にroute[0]が実際の現在地と一致しなくなり、
            # resolve_route側でbridgeできず機体が一瞬back-warpする別の
            # バグになる)。その代わりlast_ride_route側に残っている
            # ので、そちらから補って「戻る方向」を除外する。
            prev_node = plan.last_ride_route[-2]

        # get_turn_options_along_ride(hold_short付き)を先に入れておくことで、
        # 末端ノード(=別TaxiWayとの交差点そのものが終点になっている場合)
        # についても、continue_routes(hold_shortなし)と経路が重複した際に
        # hold_short付きの方が優先して残るようにする。
        entries = []
        if plan.last_ride_route:
            ride = plan.last_ride_route
            ride_entry_corner = None
            if len(ride) > 1:
                ride_entry_corner = _incoming_corner(aircraft, plan.path, ride[0], ride[1])
            entries += get_turn_options_along_ride(ride, entry_corner=ride_entry_corner)

        tail_entry_corner = None
        if prev_node is not None:
            tail_entry_corner = _incoming_corner(aircraft, plan.path, prev_node, tail)

        continue_routes = get_reachable_routes_from(
            tail,
            heading,
            exclude_prev=prev_node,
            approach_remaining=math.inf,
            entry_corner=tail_entry_corner,
        )
        entries += [(name, route, None) for name, route in continue_routes]

        seen = set()
        deduped = []
        for taxiway_name, route, hold_short_route in entries:
            key = tuple(route)
            if key in seen:
                continue
            seen.add(key)
            deduped.append((taxiway_name, route, hold_short_route))

        return deduped

    if plan.revise and aircraft.legs:
        # Revise: 今の物理位置から見て、まだコミットしていない直近の
        # ノードを基準に検索する(Continue Taxiのように今のleg全体の
        # 終点=予約先を基準にはしない)。
        node, approach_leg, approach_progress, heading, entry_corner = (
            locate_current_position(aircraft.legs[0], aircraft.leg_progress)
        )
        exclude_prev = leg_entry_node(approach_leg)
        approach_remaining = approach_leg[2].length - approach_progress
    else:
        node, heading, exclude_prev, approach_remaining = _effective_planning_state(aircraft)
        entry_corner = _effective_entry_corner(aircraft)

    continue_routes = get_reachable_routes_from(
        node,
        heading,
        exclude_prev=exclude_prev,
        approach_remaining=approach_remaining,
        entry_corner=entry_corner,
    )

    if plan.pending_first_screen_hold_short and exclude_prev is not None:
        # Revise後、またはHold Position後の「最初のルート選択画面」
        # だけの特例: 次のTaxiWayへの候補を並べるのではなく、今まさに
        # 乗っているTaxiWay自体を、その本当の終点まで一気に進んだ
        # 1本のルートとして候補に出す(get_full_ride)。
        #
        # これを選ぶと、次の画面は(last_ride_routeを介して)
        # get_turn_options_along_rideがそのまま使われる。この関数は
        # 元々「rideの終点(route[-1])も、そこで別のTaxiWayへ曲がれる
        # 場所として対象に含める」設計になっているため、Mのように
        # 短いTaxiWayでも、その終点(=次のTaxiWayとの交差点)での
        # 「曲がる」「hold short of X」の両方が新しい探索ロジックなしで
        # 自動的に出てくる。
        #
        # (以前の実装は、この終点[node]自体をhold shortの停止位置
        # として扱おうとしたが、nodeは既に「実質到達済み」の論理位置
        # として扱われるため、そこより手前で止まるという選択肢は
        # そもそもこの経路システムには存在しない。正しくは、Mを選んだ
        # 「次の」画面でM1側にhold shortをかける、という形になる。)
        taxiway_name, current_ride_route = get_full_ride(exclude_prev, node)
        return [(taxiway_name, current_ride_route, None)]

    return [(name, route, None) for name, route in continue_routes]


def get_max_incoming_corner_trim(via_node, from_node):
    """via_node に from_node 方向から進入する場合、CORNER_CONNECTORSに
    定義されている(どの方面へ曲がるかに関わらず)フィレットの中で、
    via_node手前を直線のまま進む区間(t)の最大値を返す。
    同じ(via_node, from_node)の組み合わせでも、曲がる先(z)によって
    t の値が異なることがある(例: Bで3から来た場合、Eへ抜けるフィレットは
    t=80だが、Aへ抜けるフィレットはt=40)。停止位置を決める時点では
    まだどちらへ曲がるか決まっていないため、安全側に倒して
    最大値を使う。該当するフィレット定義が無ければ0.0を返す。"""
    max_t = 0.0
    for (via, y, _z), corner in corner_segments.items():
        if via == via_node and y == from_node:
            max_t = max(max_t, corner.t)
    return max_t


def get_destination_buttons(selected):
    # ボタンは固定のスクリーン(UI)座標に配置するのでズームの影響を受けない
    #
    # 各ルートにつき、状況に応じて1〜2個ボタンを出す:
    #  - "route"     : そのTaxiWayの本当の終点まで一気に進む
    #  - "hold_short": 「hold short of X」。今乗っているTaxiWay上のまま、
    #                  Xとの交差点の手前(SAFE_DISTANCE等)で止まる。
    #                  本来の "hold short of X" の意味(Y上にいてXとの
    #                  交差点方面に向かっているとき、その手前で止まる)
    #                  に合わせて、これは「別のTaxiWayに曲がる」選択肢
    #                  (get_turn_options_along_ride由来)にだけ付く。
    #                  まだ何にも乗っていない状態で選ぶ最初のTaxiWayや、
    #                  末端からの新しい選択(hold_short_routeがNone)には
    #                  hold_shortボタンは出さない。
    #                  (なお、"route" を選んで到着する場合も含め、
    #                  legsが空になって待機に入る停止は常に交差点の
    #                  手前で止まる。hold_shortボタンとの違いは
    #                  「今乗っているTaxiWayの途中で早めに止まるか、
    #                  最後まで乗るか」という経路の選び方だけ。)
    if selected is None:
        return []

    # Revise中、機体の状況が変わって前提が崩れていたら計画を破棄する。
    # ここで破棄されればgate_passedもFalseに戻るので、下の分岐で
    # 自然に1ページ目のボタンに戻る。
    selected.plan.refresh()

    if not selected.plan.gate_passed:
        # 1ページ目: Continue Taxi(今のルートの末端からの予約)。
        # 移動中(=まだ現在のlegを進行中)であれば、それに加えて
        # Revise(今の物理位置から見直して、今のルートを取りやめて
        # 別ルートへ変更する)も選べるようにする。
        y = 100
        continue_rect = pygame.Rect(600, y, 170, 30)
        buttons = [("continue_taxi", continue_rect, "Continue Taxi", None)]
        if selected.legs:
            y += 35
            revise_rect = pygame.Rect(600, y, 170, 30)
            buttons.append(("revise", revise_rect, "Revise", None))
    else:
        buttons = []
        routes = get_reachable_routes(selected)

        y = 100
        for taxiway_name, route, hold_short_route in routes:
            route_rect = pygame.Rect(600, y, 170, 30)
            buttons.append(("route", route_rect, taxiway_name, route))
            y += 35

            if hold_short_route is not None:
                hold_rect = pygame.Rect(600, y, 170, 30)
                hold_label = f"hold short of {taxiway_name}"
                buttons.append(("hold_short", hold_rect, hold_label, hold_short_route))
                y += 35

            y += 5  # ルートごとの区切りの余白

        if selected.plan.active:
            # 選択終了ボタン。ここまでにためた計画をそのまま移動経路として確定する。
            # 機体が移動中(まだ現在のlegを進行中)でも計画を組み立てられるように
            # なったため、target_nodeではなくplan.active(計画が積まれているか)
            # で判定する。
            finish_rect = pygame.Rect(600, y + 10, 170, 30)
            buttons.append(("finish", finish_rect, "Done", None))

    # Hold Position: ページ(1ページ目/2ページ目)に関わらず常時表示する、
    # ルート候補ボタンとは別枠の固定位置のボタン。押すと現在地で
    # hold short状態に確定する(Aircraft.hold_position参照)。
    hold_position_rect = pygame.Rect(600, 20, 170, 30)
    buttons.append(("hold_position", hold_position_rect, "Hold Position", None))

    return buttons


# --- 計画済みルートを、実際に移動可能な形(legs等)へ組み立てる ---

def bridge_from_approach(current_node, approach_leg, approach_progress, route):
    """hold shortで待機中(approach_legが設定されている)に新しい
    経路(route)を選んだ場合、そのままbuild_movement_legs(route)
    してしまうと、現在の(物理的にまだ手前で止まっている)位置から
    ノードの座標へ瞬間移動(warp)してしまう。

    approach_leg は、待機の原因になった元のleg (start_node, end_node,
    segment) そのもの。直線のSegmentでも、交差点をまたいだ複合カーブ
    (CornerSegment、チェーンされたフィレット含む)でも構わない。

    approach_leg の終点(end_node)が現在の route[0](=current_node)と
    一致する場合だけbridgeする。それ以外(整合しない特殊なケース)
    では、安全のためbridgeを諦めて route をそのまま使う。

    実際にapproach_legをそのまま使うか、それとも直前ノードを起点に
    ローカルな辺として組み直すか(はるか昔の履歴を捨てるか)の判断は
    build_movement_legs側で行う(entry_progressを渡してそちらに委ねる)。

    戻り値は (legs, start_progress)。legs は既に leg 単位に組み立て
    済みのリストなので、呼び出し側が改めて build_movement_legs を
    呼ぶ必要はない。"""
    if (
        approach_leg is not None
        and route
        and route[0] == current_node
        and approach_leg[1] == current_node
    ):
        return build_movement_legs(
            list(route), entry_leg=approach_leg, entry_progress=approach_progress
        )

    return build_movement_legs(list(route))


class RouteResolution:
    """resolve_route の結果。Aircraft側がそのまま自身の物理状態
    (legs, leg_progress, heading, 停止位置)に適用する。"""

    __slots__ = ("legs", "leg_progress", "heading", "final_stop_length")

    def __init__(self, legs, leg_progress, heading, final_stop_length):
        self.legs = legs
        self.leg_progress = leg_progress
        self.heading = heading
        self.final_stop_length = final_stop_length


def resolve_route(current_node, approach_leg, approach_progress, route):
    """route: 現在ノードを含む、これから進むノードIDの並び
    (route[0] == current_node を想定)を、実際に移動を開始できる形
    (legs・向き・停止位置)に組み立てる。

    hold short直後(まだノードの手前に物理的に留まっている状態)
    から新しい経路を選んだ場合は、bridge_from_approachを使って
    止まっていたleg自体を先頭に継ぎ足し、まだ進んでいない分の距離
    (approach_progress)から続きを滑らかに進む(warpしない)。

    legsの最後のleg(routeの終点ノードに到着するleg)は、常に
    終点ノードの手前で止まるようにする(final_stop_length参照)。
    止まる位置は、その交差点にその進入方向からのCORNER_CONNECTORS
    (フィレット)のうち最も遠いt(直線のまま進む区間)の点から、
    さらにSAFE_DISTANCEだけ手前(交差点・進入方向の組み合わせごとに
    ただ1つに決まる)。ノードの座標にピッタリ乗って止まることはない。
    """
    legs, start_progress = bridge_from_approach(
        current_node, approach_leg, approach_progress, route
    )

    final_stop_length = None
    heading = None

    if legs:
        _start_node, _end_node, segment, seg_reversed = legs[0]
        _, heading = segment.point_and_heading(start_progress, reversed=seg_reversed)

        # legが交差点をまたいだ複合カーブの場合もあるので、
        # 進入方向は「最後のlegの開始ノード」ではなく、渡された
        # route(bridge前)上でlast_endの直前に来るノードを使う。
        _last_start, last_end, last_segment, _last_reversed = legs[-1]
        approach_from_node = route[-2] if len(route) >= 2 else _last_start
        corner_trim = get_max_incoming_corner_trim(last_end, approach_from_node)
        # 停止位置は「その進入方向のフィレットの最も遠い点(t)から
        # さらにSAFE_DISTANCE手前」の1点(交差点・進入方向ごとに一意)。
        stop_distance = corner_trim + SAFE_DISTANCE

        final_stop_length = max(0.0, last_segment.length - stop_distance)

    return RouteResolution(legs, start_progress, heading, final_stop_length)


class FlightPlan:
    """機体ごとの『計画(キュー)』の状態を管理する。

    選択終了ボタンを押すまでの一時的な状態(積み増し中のルート)を
    持ち、確定(confirm)されたら route を返す。実際にその計画を
    移動へ反映する(resolve_route を呼んでlegsを作る)のは
    呼び出し側(Aircraft.set_route)の仕事。

    owner には Aircraft インスタンスを渡す。current_node / heading /
    approach_leg / approach_progress を読むためだけに参照する
    (movement.py はAircraftクラス自体をimportしないので、ここでは
    ダックタイピングで済ませている)。"""

    def __init__(self, owner):
        self.owner = owner

        # 計画中のノード列。None のときは何も計画していない。
        # 計画中は path[0] == owner.current_node で、選ぶたびにノードが
        # 追加される。
        self.path = None
        # 計画の末端(実際の移動legの出口)での向き(度)。
        self.heading = None
        # 直前にaddで積み増した「ライド」(あるTaxiWayの端から端までの
        # 経路)。get_reachable_routesが、この経路の途中の交差点で
        # 曲がれる場所を選択肢として出すために使う。
        self.last_ride_route = None
        self.gate_passed = False

        # True の間は「Revise」セッション中であることを表す。
        # Continue Taxi(pass_gate)との違いは、path[0]をどう決めるか
        # (leg完走後の予約先か、今の物理位置から見た直近の未確定ノードか)
        # と、機体側のlegsを一切トリム/破棄しない(confirmされるまで
        # 実体には触れない)点。
        self.revise = False

        # True の間は、次に表示される「最初のルート選択画面」
        # (Revise後、またはHold Position後に初めてContinue Taxiを
        # 押した直後の画面)であることを表す。この画面だけは、
        # 通常のContinue Taxi後の画面と違い、直近の交差点についても
        # 「そのまま進む」に加えて「その交差点のHold Shortで停止する」
        # 選択肢を出す(get_reachable_routes参照)。
        # 最初の選択(add)が行われた時点で意味を失うのでクリアする。
        self.pending_first_screen_hold_short = False

    def pass_gate(self):
        """1ページ目の「Continue Taxi」が押されたときに呼ぶ。
        以降このセッション中は通常の経路選択肢(2ページ目、今のルートの
        末端からの予約)が表示されるようになる。
        pending_first_screen_hold_shortはここでは変更しない
        (Hold Position直後にContinue Taxiが押された場合だけ、
        mark_hold_positionで立てたフラグをそのまま持ち越して
        次の画面で使うため)。"""
        self.gate_passed = True
        self.revise = False

    def start_revise(self):
        """移動中に「Revise」が押されたときに呼ぶ。Continue Taxiと
        同じ2ページ目のUIを共有するが、path[0]を決める基準
        (movement.locate_current_position)と、確定時の反映のしかたが
        異なる(FlightPlan.add/Aircraft.set_route参照)。"""
        self.gate_passed = True
        self.revise = True
        self.pending_first_screen_hold_short = True

    def mark_hold_position(self):
        """Hold Positionが確定した直後に呼ぶ(Aircraft.hold_position
        参照)。plan.cancel()で1ページ目に戻った後、次にContinue Taxi
        が押されたときの最初のルート選択画面でだけ、直近の交差点の
        Hold Short選択肢を追加で出せるようにする。"""
        self.pending_first_screen_hold_short = True

    @property
    def active(self):
        return self.path is not None

    def refresh(self):
        """Revise中、機体の物理的な状況が変わって前提が崩れていないかを
        毎フレーム確認する。

        Reviseはaircraft.legsを一切トリムしないため、機体は選択中も
        今までのルート通り進み続ける。もし
          - legsが尽きて完全に停止してしまった(=もう選び直す途中地点が
            無くなった)
          - path[0]として選んだ『直近の未確定ノード』を、機体が実際に
            通過(コミット)してしまった
        場合、それまで組み立てていた計画はもう意味を持たないため、
        丸ごと破棄する(cancel)。これにより2ページ目のボタンは自然に
        消え、1ページ目(Continue Taxi / Revise)に戻る。"""
        if not self.revise or not self.gate_passed:
            return
        owner = self.owner
        if not owner.legs:
            self.cancel()
            return
        if self.path is not None:
            anchor, _approach_leg, _progress, _heading, _entry_corner = (
                locate_current_position(owner.legs[0], owner.leg_progress)
            )
            if anchor != self.path[0]:
                self.cancel()

    def add(self, route):
        """選んだルート(route)を計画(キュー)に追加する。

        route[0] が今の計画の末端ノードと同じ場合(普通にそのまま
        新しいTaxiWayへ進む場合)は、そのまま末尾に繋げる。

        route[0] が計画の途中のノードの場合(=すでにキューに積んで
        あった、あるTaxiWayの終点までのライドの、途中の交差点で
        曲がる選択肢を選んだ場合)は、計画のうちその交差点より先を
        切り詰めてから、新しいルートを繋げる。

        例外として、Revise後/Hold Position後の最初の画面で「今乗って
        いるTaxiWay自体」を選んだ場合に生まれるroute(get_full_ride
        の都合上、route[0]が実際の現在地ではなくその1つ手前の
        exclude_prevになっている)や、それを起点に得られる
        hold_shortルートは、route[0]自体はself.pathに存在しない。
        この場合はroute[1](=実際の現在地)を代わりの基準点として
        使う。詳細はメソッド内のコメントを参照。

        まだ計画を開始していなければここで開始する。機体がまだ現在の
        legを移動中の場合、owner._begin_replan()でそのlegだけを残して
        先の(古い経路の)legsを打ち切り、_effective_planning_stateで
        「そのlegを終えた先のノード・向き」を計画の起点にする。これに
        より、移動中の機体にも新しい指示を積み始められる。
        曲線区間(交差点の手動フィレット含む)でも正確なように、
        実際の移動単位(leg)の出口での向きを使う。"""
        owner = self.owner

        if self.path is None:
            if self.revise:
                # Reviseはowner.legsに一切触れない(確定するまで実体は
                # 今までのルート通り進み続ける)。起点は「今の物理位置
                # から見て、まだコミットしていない直近のノード」。
                node, _approach_leg, _progress, heading, _entry_corner = (
                    locate_current_position(owner.legs[0], owner.leg_progress)
                )
            else:
                owner._begin_replan()
                node, heading, _exclude_prev, _remaining = _effective_planning_state(owner)
            self.path = [node]
            self.heading = heading
            # 注意: ここでpending_first_screen_hold_shortをクリアしては
            # いけない。get_reachable_routesはself.pathがNoneのときだけ
            # このフラグを参照する(self.pathが埋まった以降の画面は
            # get_turn_options_along_ride側の仕組みがそのままHold Short
            # 選択肢を出すので、この時点ではどのみち参照されない)。
            # にもかかわらずここで先にFalseへクリアしてしまうと、この
            # 計画がconfirm()まで至らずcancel()で破棄された場合(例:
            # 選択肢を1つ選んだ直後に他の機体を選び直した、または空欄を
            # クリックして選択解除した場合)、機体の物理的な状態
            # (Hold Position/Reviseで止まったままTaxiWayの途中にいる
            # こと自体)は何も変わっていないのに、次にContinue Taxiを
            # 押したときの最初の画面で今乗っているTaxiWayの選択肢が
            # 二度と出てこなくなってしまう。実際にこのフラグが不要に
            # なるのは、計画がconfirm()され機体が本当に動き出した
            # (=物理的な状態が変わった)時点なので、クリアはconfirm()
            # 側で行う。

        # route[0]をself.pathの中の対応する位置(anchor)に結びつけて、
        # 「そのanchorより先を切り詰めてから、続きを繋げる」という
        # 単一の処理に統一する。
        #
        # 通常はroute[0]自体がself.pathのどこか(多くの場合は末尾)に
        # 一致するので、それをそのままanchorとして使う(offset=1、
        # route[1:]をpathへ足す)。
        #
        # 唯一の例外が、Revise後/Hold Position後の最初の画面で
        # 「今乗っているTaxiWay自体」を選んだ場合に生まれるrouteと、
        # それ由来のturn/hold_shortルート。これらはroute[0]が
        # 実際の現在地ノードではなく、その1つ手前(exclude_prev、
        # 既にcommit済みで、そもそもself.pathには含まれないノード)
        # になっている。この場合はroute[1](=実際の現在地。必ず
        # self.pathのどこかに存在する)をanchorとして使い、route[2:]
        # をpathへ足す(offset=2)。
        anchor = route[0]
        anchor_offset = 1
        if anchor not in self.path and len(route) > 1 and route[1] in self.path:
            anchor = route[1]
            anchor_offset = 2

        if anchor == self.path[-1]:
            self.path.extend(route[anchor_offset:])
        else:
            # 末尾からさかのぼって最後に出現した位置を探し、そこまで切り詰める
            trim_index = None
            for idx in range(len(self.path) - 1, -1, -1):
                if self.path[idx] == anchor:
                    trim_index = idx
                    break
            if trim_index is not None:
                self.path = self.path[: trim_index + 1]
            self.path.extend(route[anchor_offset:])

        # このルート自体が、次の画面で「途中の交差点で曲がれる場所」を
        # 探すための基準(last_ride_route)になる
        self.last_ride_route = route

        legs, _start_progress = bridge_from_approach(
            owner.current_node, owner.approach_leg, owner.approach_progress, self.path
        )
        if legs:
            # self.pathが2要素以上あり、実際に新しく進むlegが1つ以上
            # 生成された場合だけ、その最後のlegの出口の向きに更新する。
            _last_start, _last_end, last_segment, last_reversed = legs[-1]
            _, heading = last_segment.point_and_heading(last_segment.length, reversed=last_reversed)
            self.heading = heading
        # legsが空(self.pathが1要素のみ、つまり「今乗っているTaxiWay自体を
        # 選んだだけでまだどこにも進んでいない」場合)は、build_movement_legs
        # 側の「entry_leg合流」処理がroute長2未満では走らないため何も
        # 返らない。この場合self.headingは既に上で計算済みの値(現在地
        # での向き)のままでよいので、そのまま何もしない。

    def confirm(self):
        """選択終了ボタンが押されたら、ためた計画を確定する。
        route を返す(呼び出し側がこれを Aircraft.set_routeに渡して
        実際の移動を開始させる)。計画が無ければNoneを返す。
        呼び出し後、内部状態はリセットされる。

        self.pathが1要素だけの場合(第1画面で「今乗っているTaxiWay
        自体」を選んだ、または直後に「hold short of X」を選んだ場合)
        も、長さ1のrouteとしてそのまま返す。これは新しいノードへは
        進まないが、Hold Positionで止まった位置がまだ正式な
        hold short位置(SAFE_DISTANCE手前)ちょうどとは限らないため、
        Aircraft.set_route側でそこまで機体を進める処理が必要になる
        (movement.resolve_routeはroute長2未満からはlegを作れない
        ため、ここでは何もしないのが正しいわけではない)。"""
        result = None
        if self.path:
            result = list(self.path)
        self.cancel()
        if result is not None:
            # 実際に計画が確定し、機体がこれから物理的に動き出す
            # (Aircraft.set_routeが呼ばれる)場合だけ、Hold Position/
            # Reviseに由来する『今乗っているTaxiWayの選択肢を出す』
            # 状態を消費し尽くしたことにする。単に計画を組み立てて
            # 破棄しただけ(cancel()経由)ではここに来ないので、
            # 物理的な状態が変わっていない限りフラグは残り続ける。
            self.pending_first_screen_hold_short = False
        return result

    def cancel(self):
        """選択を確定せずに計画を破棄する(他機体を選び直した場合など)。
        次にこの機体へ指示を出すときは、必ずまた1ページ目
        (Continue Taxi)から始まるようにgate_passedもリセットする。

        pending_first_screen_hold_shortはここではリセットしない。
        このフラグはHold Position/Reviseで止まった際の『物理的な状態』
        (今まさにTaxiWayの途中にいる)を表すもので、add()で実際に
        選択肢を1つ選んだ瞬間に初めて消費・クリアされる(add参照)。
        ここでcancel()のたびに毎回リセットしてしまうと、何も選ばずに
        単に選択解除しただけ(他機体をクリック/空クリックで選択を
        外した場合など)でもフラグが失われ、再選択してContinue Taxi
        を押したときに今乗っているTaxiWayの選択肢が出なくなってしまう
        (実際の位置は何も変わっていないのに)。"""
        self.path = None
        self.heading = None
        self.last_ride_route = None
        self.gate_passed = False
        self.revise = False


def build_preview_legs(aircraft):
    """機体が計画中(plan.path)のルートを、実際の移動単位(leg)に
    変換して返す。graph.draw_graph に渡すことで、まだ確定していない
    計画中の経路を青い線でプレビュー表示できる。
    計画が無ければNoneを返す。

    hold short中(またはまだ現在のlegを移動中)にplan.path[0]自体で
    曲がる計画を組んだ場合も、実際の移動(resolve_route)と同じく
    bridge_from_approachを通すことで、approach_leg(進入方向)を
    踏まえた正しい形(直線を挟まない複合カーブ)でプレビューできる。"""
    if aircraft is None:
        return None
    plan = aircraft.plan
    if not plan.path or len(plan.path) <= 1:
        return None
    legs, _start_progress = bridge_from_approach(
        aircraft.current_node, aircraft.approach_leg, aircraft.approach_progress, plan.path
    )
    return legs

def _effective_entry_corner(aircraft):
    """_effective_planning_state が返す基準ノードへの到着で、実際に
    フィレット(角)を使ったかどうかを corner_segments のキーとして返す。

    plan.path がまだ無い(=何も計画を積み増していない)状態で
    get_reachable_routes が選択肢を組み立てる際、直前に使ったフィレットと
    物理的に連結できない角を弾けるようにするためのもの。_incoming_corner
    が plan.path をベースに行っているのと同じ判定を、まだ計画が無い
    (停止中/移動中どちらもありうる)状態向けに行う。

    aircraft.legs[0](移動中)または aircraft.approach_leg(停止中)の
    segment が CornerSegment であれば、そこで実際に使われた最後の角の
    キー (via, y1, to_node) を返す。via は leg_entry_node と同じく
    segment.node_x(ノードIDの再照合なしに求まる、直近に曲がった角)。
    y1(そのviaへ入ってきた直前のノード)は、CornerSegment.edge_yx を
    1段だけ辿って求める:
      - edge_yx 自体が CornerSegment(さらに前の角を含むチェーン)なら、
        その node_x が y1(その角も自分で組み立てた際に構造的に
        確定させたものなので、ここでもID比較は発生しない)。
      - edge_yx が生の Segment(両端ノードIDが必ず異なる)なら、
        via ではない方の端点が y1(ここでのID比較は生の1区間に対して
        だけなので安全)。

    まだ一度もフィレットを使っていない(segment が生のSegmentのまま)場合や、
    今のところ計画も何も進行中のlegも無い場合はNoneを返す。"""
    if aircraft.legs:
        leg = aircraft.legs[0]
    elif aircraft.approach_leg is not None:
        leg = aircraft.approach_leg
    else:
        return None

    _start_node, to_node, segment, _seg_reversed = leg
    via = getattr(segment, "node_x", None)
    if via is None:
        return None  # 生のSegment。まだ角を使っていない

    edge_yx = segment.edge_yx
    y1 = getattr(edge_yx, "node_x", None)
    if y1 is None:
        y1 = edge_yx.node_a if edge_yx.node_a != via else edge_yx.node_b

    key = (via, y1, to_node)
    return key if key in corner_segments else None