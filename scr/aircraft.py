import math
import pygame

from settings import SAFE_DISTANCE, WIDTH, HEIGHT
from graph import nodes
import movement


class Aircraft:
    """1機の航空機。

    経路の『計画』(どこへ進むかの選択・積み増し・確定)は
    movement.FlightPlan に委譲し、Aircraft自身は
      - 計画を movement.py に求める(add_to_plan / confirm_plan / cancel_plan)
      - 確定した計画(legs)に従って、実際にフレームごとに座標・向きを
        更新する(update)
      - 現在の状態を画面に描画する(draw)
    ことだけを担当する。

    【移動中の再計画について】
    完全に停止していなくても(まだ現在のlegを移動中でも)、新しい
    計画を組み立て始められる。計画の最初の一手(FlightPlan.add)が
    積まれた瞬間、_begin_replanが呼ばれ、現在進行中のleg1つだけを
    残して先の(古い経路の)legsを打ち切り、そのlegの終端で自然に
    hold shortするよう停止距離を計算し直す。以降は下記の通常の
    hold shortの仕組みがそのまま働くので、Aircraft.update自体は
    変更不要。

    【停止(hold short)について】
    機体が次の指示待ちで完全に静止するとき(legsが空になるとき)は、
    常に交差点の手前で止まる。停止位置は、その進入方向のフィレット
    (t/s)のうち最も遠い点からさらにSAFE_DISTANCE手前という、交差点・
    進入方向ごとにただ1つの位置になる(実際の距離の計算は
    movement.resolve_routeが行う)。ノードの座標にピッタリ乗って
    止まることはない。
    交差点をまたいだ複合カーブ(CornerSegment)の途中で止まった場合も
    含め、止まった原因のleg(approach_leg)とその中での進捗
    (approach_progress)を覚えておくことで、次に新しい経路が
    指示されたときにワープせず滑らかに続きを進められる
    (movement.bridge_from_approach参照)。current_node自体は
    論理的にはすでにそのlegの終点ノードへ更新済みなので、次の指示は
    そのノードを起点として選べる。"""

    def __init__(self, name, node_id, heading=90.0):
        self.name = name
        self.current_node = node_id

        # これから移動する『leg』のリスト。各leg は
        # (start_node, end_node, segment, reversed)。reversedは、
        # segment.point_and_heading(distance, reversed=reversed) を
        # 呼ぶとstart_node起点の位置が返ってくることを表すフラグ
        # (movement.build_movement_legs側で構築時に確定させたもの。
        # 進行方向の判定にノードIDの一致は使わない)。
        # 交差点に手動フィレットが定義されている場合、legは2つ分のノードを
        # 1本の曲線としてまとめて表す(途中のノードでは止まらない)。
        self.legs = []
        # 現在のleg(legs[0])を、その開始点から何ワールド距離単位進んだか
        self.leg_progress = 0.0

        self.x, self.y = nodes[node_id]
        self.speed = 0.1
        # 機体の向き(度)。0 = 右向き。前後の概念はこの角度で表現する。
        self.heading = heading

        # 選択終了ボタンを押すまでの計画(キュー)の状態は
        # movement.FlightPlan が持つ。
        self.plan = movement.FlightPlan(self)

        # 現在の経路(self.legs)の最終legが、終点ノードまで完全には
        # 進まず、その手前で止まる(hold short)ときの、弧長上の
        # 目標距離(legs[0]のsegmentのstart_nodeから)。
        self._final_stop_length = None

        # hold shortで止まった結果、current_nodeは更新済みだが物理的には
        # そのlegの終点までまだ到達していない場合の情報。
        # approach_leg: 止まった原因になったleg (start_node, end_node,
        #               segment) そのもの。直線でも、交差点をまたいだ
        #               複合カーブ(CornerSegment)でもよい。
        # approach_progress: そのleg内で、start_nodeから実際に進んだ距離
        #               (=止まった位置)。
        # ノードにまだ一度も到着していない(=出発直後)の間はNone/0.0。
        self.approach_leg = None
        self.approach_progress = 0.0

    @property
    def target_node(self):
        return self.legs[0][1] if self.legs else None

    @property
    def planned_path(self):
        """計画中(未確定)のノード列。movement.FlightPlan.path のエイリアス。"""
        return self.plan.path

    @property
    def approach_from(self):
        """待機中(approach_leg有り)なら、来た方向のノード。待機中で
        なければNone。get_reachable_routes等が参照する後方互換の
        プロパティ。

        approach_legが交差点をまたいだ複合カーブ(CornerSegmentを
        _build_chained_cornerでチェーンしたもの)の場合、その
        start_nodeはチェーン全体の起点(遠く離れたノード)であり、
        直前に通過したノードではない。そのため単純にapproach_leg[0]
        を返すと、複合カーブ経由で停止した際にfrom_nodeがいつまでも
        移動前の最初のノードのままになり、_is_turn_allowedでの直進
        判定・フィレット判定が正しい隣接ノード同士で行われず、選択肢が
        一切出なくなってしまう。movement.leg_entry_nodeが同じ問題を
        既に正しく扱っている(CornerSegment.node_xを見る)ので、
        ここでもそれを使う。"""
        if self.approach_leg is None:
            return None
        return movement.leg_entry_node(self.approach_leg)

    @property
    def approach_remaining(self):
        """待機中(approach_leg有り)なら、現在の物理的な位置から
        current_nodeまでの残り距離。待機中でなければmath.inf。"""
        if self.approach_leg is None:
            return math.inf
        return self.approach_leg[2].length - self.approach_progress

    # --- 計画(選択肢を積み増す/確定する/破棄する)は movement.py に委譲 ---

    def _begin_replan(self):
        """新しい計画の組み立てを開始する瞬間(FlightPlan.addが最初に
        呼ばれた瞬間)に呼ばれる。まだ現在のlegを移動中(self.legsが
        複数、または途中まで進んでいる)であれば、進行中のleg1つだけ
        を残して先の(古い経路の)legsを打ち切り、そのlegの終端で
        自然にhold shortするよう_final_stop_lengthを計算し直す。

        これにより、以降はupdate側の既存のhold shortの仕組み
        (legsが空になり、approach_leg/approach_progressで続きを
        繋ぐ)がそのまま働くようになる。実際に指示が確定する
        (set_route)までの間、機体はこの1leg分の残りだけを進み続け、
        それ以上(古い経路の続き)には進まない。

        停止距離は通常のhold shortと同じ式(その進入方向のフィレット
        の最も遠い点 + SAFE_DISTANCE)で求めるが、既にその位置を
        過ぎている(元の経路ではこのlegをそのまま通過する予定
        だった等)場合は、後退させて瞬間移動させないよう、
        少なくとも現在の進捗までは進めてから止める。

        legsが既に空(=既に完全に停止している)なら何もしない。"""
        if not self.legs:
            return

        self.legs = self.legs[:1]
        _start_node, end_node, segment, _seg_reversed = self.legs[0]

        approach_from_node = movement.leg_entry_node(self.legs[0])
        corner_trim = movement.get_max_incoming_corner_trim(end_node, approach_from_node)
        stop_distance = corner_trim + SAFE_DISTANCE
        natural_stop = max(0.0, segment.length - stop_distance)

        self._final_stop_length = max(natural_stop, self.leg_progress)

    def add_to_plan(self, route):
        self.plan.add(route)

    def confirm_plan(self):
        """選択終了ボタンが押されたら、ためた計画を movement.py で確定し、
        その結果(route)を実際の移動経路として反映する。

        plan.confirm()はplan.cancel()を内部で呼んで revise フラグ含む
        計画の状態をリセットしてしまうので、それより前に読み取っておく。"""
        was_revise = self.plan.revise
        route = self.plan.confirm()
        if route is not None:
            self.set_route(route, revise=was_revise)

    def cancel_plan(self):
        """選択を確定せずに計画を破棄する(他機体を選び直した場合など)"""
        self.plan.cancel()

    def hold_position(self):
        """『今いる位置』をhold short状態として確定する(Hold Position)。

        新しい状態表現や幾何計算は追加せず、movement.locate_current_position
        が返すhold short時と同じ状態表現(current_node, approach_leg,
        approach_progress, heading)への変換をそのまま利用する。これにより、
        以降はContinue Taxi・ルート候補生成・confirm_plan・set_route・
        resolve_route・bridge_from_approachといった既存のhold short再開の
        仕組みが、通常の停止時と全く同じように動く。

        1. 組み立て中の計画を破棄する。
        2. self.legsが残っている(移動中、またはContinue Taxi/Reviseで
           計画を組み立てている途中)場合のみ、現在のleg(self.legs[0])と
           その中での進捗(self.leg_progress)から実際の物理位置を求め、
           hold short中の機体が持つのと同じ属性
           (current_node/approach_leg/approach_progress/heading)へ反映する。
           entry_cornerは既存のhold short処理と同様、Aircraft側では保存
           せず、_effective_entry_cornerがapproach_legのsegment構造
           (node_x)から都度再導出するのに任せる。
        3. self.legsが既に空(=既に完全に停止している)ならこの変換は不要
           なので、そのままlegs/leg_progress/_final_stop_lengthを完全停止
           状態にリセットするだけでよい。
        4. plan.cancel()の後にplan.mark_hold_position()を呼び、次に
           Continue Taxiが押されたときの最初のルート選択画面でだけ、
           直近の交差点のHold Short選択肢を追加で出せるようにする
           (movement.FlightPlan.mark_hold_position / get_reachable_routes
           参照)。
        """
        self.plan.cancel()
        self.plan.mark_hold_position()

        if self.legs:
            current_node, approach_leg, approach_progress, heading, _entry_corner = (
                movement.locate_current_position(self.legs[0], self.leg_progress)
            )
            self.current_node = current_node
            self.approach_leg = approach_leg
            self.approach_progress = approach_progress
            self.heading = heading

        self.legs = []
        self.leg_progress = 0.0
        self._final_stop_length = None

    # --- 確定した経路を、実際の移動状態に反映する ---

    def set_route(self, route, revise=False):
        """route: これから進むノードIDの並び(route[0]は、機体が
        既に停止していればcurrent_node、まだ現在のlegを移動中なら
        そのlegを終えた先のノード、reviseの場合は今の物理位置から見て
        まだコミットしていない直近のノードを想定)。

        まだ現在のlegを移動中(self.legsが残っている)場合は、
        まずそのlegをapproach_leg/approach_progressとして確定させる。

        - 通常(Continue Taxi等、revise=False): _begin_replanで既に
          1leg分だけに切り詰め済みのはずなので、単にそれを『待機中』の
          状態と同じ形に変換するだけでよい。
        - revise=True: legsには一切触れていない(movement.FlightPlan.add
          参照)ので、ここで初めて、確定した"今この瞬間"の実際の物理位置
          (movement.locate_current_positionで今のleg_progressから求め
          直す)を起点として採用する。plan構築中にため込んだ古い経路の
          続き(self.legsの残り)はここで丸ごと捨てられる。

        どちらの場合も、movement.resolve_route の結果(legs・向き・
        停止位置)を自分の物理状態にそのまま反映する。『移動を開始する
        瞬間』にだけ回転する。"""
        if self.legs:
            if revise:
                current_node, approach_leg, approach_progress, _heading, _entry_corner = (
                    movement.locate_current_position(self.legs[0], self.leg_progress)
                )
                self.approach_leg = approach_leg
                self.approach_progress = approach_progress
                self.current_node = current_node
            else:
                _start_node, end_node, _segment, _seg_reversed = self.legs[0]
                # legs[0]は既に(start_node, end_node, segment, reversed)の
                # 4要素タプルなので、そのままapproach_legとして使い回せる。
                self.approach_leg = self.legs[0]
                self.approach_progress = self.leg_progress
                self.current_node = end_node
            self.legs = []
            self.leg_progress = 0.0
            self._final_stop_length = None

        if len(route) <= 1:
            # route長1(=今乗っているTaxiWay自体、またはその
            # hold short of X を選んで、新しいノードへは進まない
            # まま確定した場合)。movement.resolve_route/
            # build_movement_legsは2ノード未満のrouteからはleg
            # (=区間)を作れないため、代わりに_begin_replanと同じ式
            # (進入方向のフィレットの最も遠い点 + SAFE_DISTANCE)で、
            # 今のapproach_leg上の『正しいhold short停止位置』まで
            # 機体を進める。
            #
            # Hold Positionはボタンを押した瞬間の物理位置でそのまま
            # 止めるだけ(SAFE_DISTANCEを踏まえた正式な停止位置とは
            # 限らない)ので、ここで改めて正しい位置まで滑らかに
            # 進める必要がある。
            self._resume_to_safe_hold_short()
            return

        resolution = movement.resolve_route(
            self.current_node,
            self.approach_leg,
            self.approach_progress,
            route,
        )

        self.legs = resolution.legs
        self.leg_progress = resolution.leg_progress
        self._final_stop_length = resolution.final_stop_length
        # bridgeで使ったapproach情報はここで消費し終わったのでリセットする
        self.approach_leg = None
        self.approach_progress = 0.0
        if resolution.heading is not None:
            self.heading = resolution.heading

    def _resume_to_safe_hold_short(self):
        """set_routeがroute長1(今乗っているTaxiWay自体を確定しただけ、
        新しいノードへは進まない)を受け取ったときに呼ぶ。

        self.approach_leg/self.approach_progress(このメソッドが
        呼ばれる時点で、set_route冒頭のself.legsトリム処理により
        必ず『待機中』の形に揃っている)から、_begin_replanと全く
        同じ式で正しいhold short停止位置(natural_stop)を求め、
        self.legsへ1leg分だけ差し戻して、通常のupdate()の移動
        アニメーションでそこまで滑らかに進ませる(現在の進捗より
        後退はさせない)。approach_legが無い(出発直後などで
        まだ何のTaxiWayにも乗っていない)場合は何もしない。"""
        approach_leg = self.approach_leg
        if approach_leg is None:
            return

        approach_progress = self.approach_progress
        _start_node, end_node, segment, _seg_reversed = approach_leg
        approach_from_node = movement.leg_entry_node(approach_leg)
        corner_trim = movement.get_max_incoming_corner_trim(end_node, approach_from_node)
        stop_distance = corner_trim + SAFE_DISTANCE
        natural_stop = max(0.0, segment.length - stop_distance)

        self.legs = [approach_leg]
        self.leg_progress = approach_progress
        self._final_stop_length = max(natural_stop, approach_progress)
        self.approach_leg = None
        self.approach_progress = 0.0

    def update(self, aircrafts):
        if not self.legs:
            return

        _start_node, end_node, segment, seg_reversed = self.legs[0]

        # 現在のlegが最終legの場合、segment.lengthまでではなく、
        # 交差点の手前で止まる距離(_final_stop_length)を目標にする。
        # legsが空になる停止は常にこの「手前で止まる」形になる。
        is_final_leg = len(self.legs) == 1
        target_length = self._final_stop_length if is_final_leg else segment.length

        next_progress = self.leg_progress + self.speed

        if next_progress >= target_length:
            if not is_final_leg:
                # 途中のlegの終点ノードに到着する場合。到着する位置で
                # 安全距離を確認する。
                end_pos, _ = segment.point_and_heading(segment.length, reversed=seg_reversed)
                for other in aircrafts:
                    if other is self:
                        continue
                    d = math.hypot(end_pos[0] - other.x, end_pos[1] - other.y)
                    if d < SAFE_DISTANCE:
                        return  # 安全距離を保つため今フレームは待機

                self.x, self.y = end_pos
                self._arrive(end_node)
                return

            # 最終leg: 交差点の手前(target_length)で待機を開始する。
            # ここでlegsを空にすることで、target_nodeがNoneに戻り、
            # 次の指示(get_reachable_routes)を受け付けられるように
            # なる。次の計画は、この待機地点の手前ノード(end_node)
            # から出ているものとして扱う(実際の見た目上の位置は
            # end_nodeの手前のまま)。
            #
            # approach_leg/approach_progressに、止まった原因のleg
            # そのものと、その中での進捗を覚えておく。これにより、
            # 次に新しい経路を選んだ際、直線でも交差点をまたいだ
            # 複合カーブでも、ここから瞬間移動(warp)せずに続きを
            # 滑らかに進められる(movement.bridge_from_approach参照)。
            stop_pos, stop_heading = segment.point_and_heading(target_length, reversed=seg_reversed)
            for other in aircrafts:
                if other is self:
                    continue
                d = math.hypot(stop_pos[0] - other.x, stop_pos[1] - other.y)
                if d < SAFE_DISTANCE:
                    return  # 安全距離を保つため今フレームは待機

            self.x, self.y = stop_pos
            self.heading = stop_heading
            self.current_node = end_node
            self.approach_leg = (_start_node, end_node, segment, seg_reversed)
            self.approach_progress = target_length
            self.legs = []
            self.leg_progress = 0.0
            self._final_stop_length = None
            return

        # 曲線でも直線でも、弧長ベースで一定速度に進む。
        # 向き(heading)はその位置での接線方向になるので、
        # 曲線区間(交差点のフィレット含む)では移動中に連続的に回転する。
        next_pos, next_heading = segment.point_and_heading(next_progress, reversed=seg_reversed)

        for other in aircrafts:
            if other is self:
                continue
            d = math.hypot(next_pos[0] - other.x, next_pos[1] - other.y)
            if d < SAFE_DISTANCE:
                return  # 安全距離を保つため今フレームは待機

        self.x, self.y = next_pos
        self.heading = next_heading
        self.leg_progress = next_progress

    def _arrive(self, node_id):
        """途中のlegの終端(交差点、または手動フィレットで飛ばした先の
        ノード)に到着した処理。続きのlegが残っている場合は、止まらずに
        そのまま次のlegへ向けて回転・移動を続ける。
        (legsが空になる=待機に入る停止は、常にupdate側の『最終leg』
        分岐(手前で止まる方)で扱われるので、ここには来ない。)
        物理的にノードちょうどの座標へ到着しているので、approach_leg/
        approach_progressは使わない(素のNone/0.0のまま)。"""
        self.current_node = node_id
        self.legs.pop(0)
        self.leg_progress = 0.0
        if self.legs:
            _start_node, _end_node, segment, seg_reversed = self.legs[0]
            _, heading = segment.point_and_heading(0.0, reversed=seg_reversed)
            self.heading = heading

    def draw(self, screen, font, camera, selected=False):
        # ワールド座標(self.x, self.y)を、ここで初めてスクリーン座標に変換する。
        # 移動ロジック(update)は一切ズームの影響を受けない。
        sx, sy = camera.world_to_screen(self.x, self.y, WIDTH, HEIGHT)

        color = (255, 220, 80) if selected else (255, 255, 255)

        # 大きさはズームに応じて見た目だけ変える
        size = 12 * camera.zoom
        pygame.draw.rect(
            screen,
            color,
            (int(sx - size / 2), int(sy - size / 2), max(1, int(size)), max(1, int(size))),
        )
        pygame.draw.circle(
            screen,
            color,
            (int(sx), int(sy)),
            max(1, int(SAFE_DISTANCE / 2 * camera.zoom)),
            2,
        )

        label = font.render(self.name, True, (200, 200, 200))
        screen.blit(label, (sx - 20, sy - 30))