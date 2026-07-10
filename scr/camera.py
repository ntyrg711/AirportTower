class Camera:
    """ワールド座標(距離の単位。ピクセルとは独立)と
    スクリーン座標(ピクセル)を変換するクラス。

    ノードの座標や機体の速度は常にワールド座標系で扱う。
    ズームやパンは描画時の変換だけに影響し、
    移動にかかる時間(ワールド距離 / 速度)には一切影響しない。
    """

    def __init__(self):
        self.zoom = 1.0
        self.min_zoom = 0.3
        self.max_zoom = 3.0

        # 画面中央に表示するワールド座標
        self.offset_x = 0.0
        self.offset_y = 0.0

    def world_to_screen(self, x, y, screen_w, screen_h):
        sx = (x - self.offset_x) * self.zoom + screen_w / 2
        sy = (y - self.offset_y) * self.zoom + screen_h / 2
        return sx, sy

    def screen_to_world(self, sx, sy, screen_w, screen_h):
        x = (sx - screen_w / 2) / self.zoom + self.offset_x
        y = (sy - screen_h / 2) / self.zoom + self.offset_y
        return x, y

    def zoom_at(self, screen_pos, screen_w, screen_h, factor):
        """screen_pos(マウス位置)を中心に拡大縮小する。
        ズーム後もカーソル下のワールド座標がずれないようにoffsetを補正する。"""
        mx, my = screen_pos
        before_x, before_y = self.screen_to_world(mx, my, screen_w, screen_h)

        new_zoom = self.zoom * factor
        self.zoom = max(self.min_zoom, min(self.max_zoom, new_zoom))

        after_x, after_y = self.screen_to_world(mx, my, screen_w, screen_h)
        self.offset_x += before_x - after_x
        self.offset_y += before_y - after_y

    def pan(self, dx_screen, dy_screen):
        """スクリーン座標上でのドラッグ量(dx_screen, dy_screen)ぶん
        表示位置を移動する。"""
        self.offset_x -= dx_screen / self.zoom
        self.offset_y -= dy_screen / self.zoom
