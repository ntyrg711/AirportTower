# -*- coding: utf-8 -*-
"""
OpenStreetMap 座標ピッカー (pygame)
=====================================

機能
----
- OpenStreetMap のタイル画像を背景として重ねて表示
- マウスドラッグでパン、マウスホイールでズーム（カーソル位置を中心にズーム）
- 「基準点(0,0)」をクリックで設定
- 任意の地点をクリックし、名前を入力して
      "名前": (X, Y)
  の形式（X=東西, Y=南北, 単位はメートル）でファイルに保存
- スケールを東西・南北で一致させるため、緯度経度をそのまま使わず
  Web Mercator（EPSG:3857, OSM タイルと同じ投影）のメートル座標に変換してから
  基準点との差分を計算している（Web Mercator は等角図法なので、ある地点の
  近傍では X 方向・Y 方向のスケールが等しくなる）

操作方法
--------
  左ドラッグ         : 地図をパン（移動）
  マウスホイール      : ズームイン・アウト（カーソル位置を中心に）
  O キー             : 「基準点設定モード」に入る。次にクリックした地点が (0,0) になる
  A キー             : 「地点追加モード」に入る。次にクリックした地点の名前を入力して保存
  Enter              : 名前入力を確定して保存
  Esc                : モードや名前入力をキャンセル
  S キー             : 手動で保存ファイルを書き出す（通常は追加のたびに自動保存）
  +/- または =/-      : キーボードでもズーム可能

必要なもの
----------
  pip install pygame
  インターネット接続（OpenStreetMap のタイルサーバーから画像を取得するため）

注意（OSMタイル利用について）
------------------------------
  OpenStreetMap の標準タイルサーバー(tile.openstreetmap.org)は個人の少量利用を
  想定したものです。大人数での利用や高頻度アクセスは利用規約違反になり得ます。
  本格運用する場合は自前のタイルサーバーや MapTiler 等の商用タイルサービスに
  切り替えてください。（本スクリプトはダウンロードしたタイルをディスクに
  キャッシュし、同じタイルを何度も取得しないようにしています）
"""

import pygame
import sys
import os
import io
import math
import ast
import urllib.request

# ============================================================
# 定数
# ============================================================
TILE_SIZE = 256
CACHE_DIR = "tile_cache"                 # タイル画像のディスクキャッシュ先
SAVE_FILE = "points.txt"                 # 座標の保存先ファイル
USER_AGENT = "pygame-osm-coordinate-picker/1.0 (personal use)"
TILE_URL_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"

MIN_ZOOM = 2
MAX_ZOOM = 19

EARTH_R = 6378137.0  # Web Mercator (EPSG:3857) の地球半径 [m]

WINDOW_W, WINDOW_H = 1000, 700

os.makedirs(CACHE_DIR, exist_ok=True)


# ============================================================
# 座標変換関数
# ============================================================
def lonlat_to_pixel(lon, lat, zoom):
    """経度緯度 -> 「ズームレベル zoom における」全世界ピクセル座標
    （OSM タイルスキームの標準的な変換式）"""
    n = 2 ** zoom
    x = (lon + 180.0) / 360.0 * n * TILE_SIZE
    lat_rad = math.radians(lat)
    y = (1.0 - math.log(math.tan(lat_rad) + 1.0 / math.cos(lat_rad)) / math.pi) \
        / 2.0 * n * TILE_SIZE
    return x, y


def pixel_to_lonlat(px, py, zoom):
    """全世界ピクセル座標 -> 経度緯度"""
    n = 2 ** zoom
    lon = px / (n * TILE_SIZE) * 360.0 - 180.0
    y_ratio = py / (n * TILE_SIZE)
    lat_rad = math.atan(math.sinh(math.pi * (1.0 - 2.0 * y_ratio)))
    lat = math.degrees(lat_rad)
    return lon, lat


def lonlat_to_meters(lon, lat):
    """経度緯度 -> Web Mercator メートル座標 (東西=x, 南北=y)
    この投影は等角図法なので、狭い範囲では x, y のスケールが等しくなる。
    """
    x = EARTH_R * math.radians(lon)
    lat_rad = math.radians(max(min(lat, 85.05112878), -85.05112878))
    y = EARTH_R * math.log(math.tan(math.pi / 4.0 + lat_rad / 2.0))
    return x, y


def meters_to_lonlat(x, y):
    """Web Mercator メートル座標 -> 経度緯度（表示用の逆変換）"""
    lon = math.degrees(x / EARTH_R)
    lat = math.degrees(2.0 * math.atan(math.exp(y / EARTH_R)) - math.pi / 2.0)
    return lon, lat


# ============================================================
# タイル管理（ダウンロード + メモリ/ディスクキャッシュ）
# ============================================================
class TileManager:
    def __init__(self):
        self.memory_cache = {}   # (zoom, x, y) -> pygame.Surface または False(取得失敗)

    def get_tile_surface(self, zoom, x, y):
        n = 2 ** zoom
        # 経度方向はループさせる（世界一周）
        x = x % n
        if y < 0 or y >= n:
            return None  # 極地より外は存在しない

        key = (zoom, x, y)
        if key in self.memory_cache:
            cached = self.memory_cache[key]
            return cached if cached is not False else None

        surface = self._load_from_disk_or_download(zoom, x, y)
        self.memory_cache[key] = surface if surface is not None else False
        return surface

    def _load_from_disk_or_download(self, zoom, x, y):
        path = os.path.join(CACHE_DIR, f"{zoom}_{x}_{y}.png")
        if not os.path.exists(path):
            url = TILE_URL_TEMPLATE.format(z=zoom, x=x, y=y)
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            try:
                with urllib.request.urlopen(req, timeout=6) as res:
                    data = res.read()
                with open(path, "wb") as f:
                    f.write(data)
            except Exception as e:
                print(f"[タイル取得失敗] z={zoom} x={x} y={y}: {e}")
                return None
        try:
            return pygame.image.load(path).convert()
        except Exception as e:
            print(f"[タイル読込失敗] {path}: {e}")
            # 壊れたキャッシュファイルは削除しておく
            try:
                os.remove(path)
            except OSError:
                pass
            return None


# ============================================================
# 日本語対応フォントの取得
# ============================================================
def get_japanese_font(size):
    candidates = [
        "notosanscjkjp", "notosanscjk", "yugothic", "msgothic",
        "meiryo", "hiraginosans", "ipagothic", "takaogothic",
    ]
    for name in candidates:
        path = pygame.font.match_font(name)
        if path:
            return pygame.font.Font(path, size)
    # 見つからない場合はデフォルトフォント（日本語は文字化けする可能性あり）
    print("[警告] 日本語対応フォントが見つかりませんでした。"
          "日本語が正しく表示されない場合はシステムに日本語フォントを"
          "インストールしてください。")
    return pygame.font.Font(None, size)


# ============================================================
# メインアプリケーション
# ============================================================
class MapApp:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_W, WINDOW_H), pygame.RESIZABLE)
        pygame.display.set_caption("OSM 座標ピッカー")
        self.clock = pygame.time.Clock()

        self.width, self.height = WINDOW_W, WINDOW_H

        self.font = get_japanese_font(20)
        self.font_small = get_japanese_font(16)

        self.tiles = TileManager()

        # 初期表示位置（東京駅付近）。好きな場所に変更可能。
        self.center_lon = 139.7671
        self.center_lat = 35.6812
        self.zoom = 15

        # 基準点 (0,0) のメートル座標。未設定なら None
        self.origin_meters = None   # (mx, my)
        self.origin_lonlat = None   # 表示用

        # 保存済み座標 { 名前: (X, Y) }
        self.points = {}
        self.load_points()

        # モード: None / "set_origin" / "add_point"
        self.mode = None

        # 名前入力用
        self.text_input_active = False
        self.text_input_buffer = ""
        self.pending_point_meters = None  # 入力確定待ちの座標

        # ドラッグ状態
        self.dragging = False
        self.drag_last_pos = (0, 0)
        self.drag_moved = False  # クリックかドラッグかの判定用

        self.status_message = "OキーとAキーで基準点/座標の登録モードに入れます"

        self.running = True

    # --------------------------------------------------------
    # 保存 / 読み込み
    # --------------------------------------------------------
    def load_points(self):
        if os.path.exists(SAVE_FILE):
            try:
                with open(SAVE_FILE, "r", encoding="utf-8") as f:
                    content = f.read().strip()
                if content:
                    data = ast.literal_eval(content)
                    if isinstance(data, dict):
                        self.points = {str(k): tuple(v) for k, v in data.items()}
            except Exception as e:
                print(f"[読み込み失敗] {SAVE_FILE}: {e}")

    def save_points(self):
        try:
            with open(SAVE_FILE, "w", encoding="utf-8") as f:
                f.write("{\n")
                for name, (x, y) in self.points.items():
                    # 名前部分はエスケープしつつダブルクオートで出力
                    safe_name = name.replace("\\", "\\\\").replace('"', '\\"')
                    f.write(f'    "{safe_name}": ({x:.3f}, {y:.3f}),\n')
                f.write("}\n")
            self.status_message = f"保存しました -> {SAVE_FILE}"
        except Exception as e:
            self.status_message = f"保存失敗: {e}"
            print(self.status_message)

    # --------------------------------------------------------
    # 座標系ヘルパー
    # --------------------------------------------------------
    def center_pixel(self):
        return lonlat_to_pixel(self.center_lon, self.center_lat, self.zoom)

    def screen_to_lonlat(self, sx, sy):
        cx, cy = self.center_pixel()
        world_x = cx - self.width / 2 + sx
        world_y = cy - self.height / 2 + sy
        return pixel_to_lonlat(world_x, world_y, self.zoom)

    def lonlat_to_screen(self, lon, lat):
        cx, cy = self.center_pixel()
        wx, wy = lonlat_to_pixel(lon, lat, self.zoom)
        sx = wx - cx + self.width / 2
        sy = wy - cy + self.height / 2
        return sx, sy

    def relative_meters_from_lonlat(self, lon, lat):
        """基準点からの相対座標 (X, Y) [m] を返す。基準点未設定なら None"""
        if self.origin_meters is None:
            return None
        mx, my = lonlat_to_meters(lon, lat)
        ox, oy = self.origin_meters
        return (mx - ox, my - oy)

    # --------------------------------------------------------
    # パン・ズーム
    # --------------------------------------------------------
    def pan_by_screen_delta(self, dx, dy):
        cx, cy = self.center_pixel()
        new_cx = cx - dx
        new_cy = cy - dy
        self.center_lon, self.center_lat = pixel_to_lonlat(new_cx, new_cy, self.zoom)

    def zoom_at(self, screen_pos, delta):
        old_zoom = self.zoom
        new_zoom = max(MIN_ZOOM, min(MAX_ZOOM, old_zoom + delta))
        if new_zoom == old_zoom:
            return
        mx, my = screen_pos
        # ズーム前: カーソル位置の経緯度を求める
        lon, lat = self.screen_to_lonlat(mx, my)
        self.zoom = new_zoom
        # ズーム後: 同じ経緯度がカーソル位置と同じ画面座標になるよう中心を調整
        wx, wy = lonlat_to_pixel(lon, lat, new_zoom)
        new_cx = wx - mx + self.width / 2
        new_cy = wy - my + self.height / 2
        self.center_lon, self.center_lat = pixel_to_lonlat(new_cx, new_cy, new_zoom)

    # --------------------------------------------------------
    # 描画
    # --------------------------------------------------------
    def draw_map(self):
        self.screen.fill((220, 220, 220))
        cx, cy = self.center_pixel()
        top_left_x = cx - self.width / 2
        top_left_y = cy - self.height / 2

        first_tile_x = int(math.floor(top_left_x / TILE_SIZE))
        first_tile_y = int(math.floor(top_left_y / TILE_SIZE))
        last_tile_x = int(math.floor((top_left_x + self.width) / TILE_SIZE))
        last_tile_y = int(math.floor((top_left_y + self.height) / TILE_SIZE))

        for tx in range(first_tile_x, last_tile_x + 1):
            for ty in range(first_tile_y, last_tile_y + 1):
                surf = self.tiles.get_tile_surface(self.zoom, tx, ty)
                screen_x = tx * TILE_SIZE - top_left_x
                screen_y = ty * TILE_SIZE - top_left_y
                if surf is not None:
                    self.screen.blit(surf, (screen_x, screen_y))
                else:
                    pygame.draw.rect(
                        self.screen, (200, 200, 200),
                        (screen_x, screen_y, TILE_SIZE, TILE_SIZE)
                    )

        # 出典表記（OSMの利用規約で表示が必須）
        attrib = self.font_small.render("(C) OpenStreetMap contributors", True, (0, 0, 0))
        attrib_bg = pygame.Surface((attrib.get_width() + 8, attrib.get_height() + 4))
        attrib_bg.fill((255, 255, 255))
        attrib_bg.set_alpha(200)
        self.screen.blit(attrib_bg, (self.width - attrib.get_width() - 12, self.height - 26))
        self.screen.blit(attrib, (self.width - attrib.get_width() - 8, self.height - 24))

    def draw_markers(self):
        # 基準点
        if self.origin_lonlat is not None:
            sx, sy = self.lonlat_to_screen(*self.origin_lonlat)
            if -20 <= sx <= self.width + 20 and -20 <= sy <= self.height + 20:
                pygame.draw.circle(self.screen, (255, 0, 0), (int(sx), int(sy)), 7)
                pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), 7, 2)
                label = self.font_small.render("基準点(0,0)", True, (255, 0, 0))
                self.screen.blit(label, (sx + 10, sy - 10))

        # 保存済み地点
        if self.origin_meters is not None:
            ox, oy = self.origin_meters
            for name, (x, y) in self.points.items():
                lon, lat = meters_to_lonlat(ox + x, oy + y)
                sx, sy = self.lonlat_to_screen(lon, lat)
                if -20 <= sx <= self.width + 20 and -20 <= sy <= self.height + 20:
                    pygame.draw.circle(self.screen, (0, 120, 255), (int(sx), int(sy)), 6)
                    pygame.draw.circle(self.screen, (255, 255, 255), (int(sx), int(sy)), 6, 2)
                    label = self.font_small.render(name, True, (0, 60, 160))
                    self.screen.blit(label, (sx + 8, sy - 8))

    def draw_ui(self):
        lines = []
        lines.append(f"ズーム: {self.zoom}")

        mx, my = pygame.mouse.get_pos()
        lon, lat = self.screen_to_lonlat(mx, my)
        lines.append(f"カーソル位置: 緯度{lat:.6f} 経度{lon:.6f}")

        rel = self.relative_meters_from_lonlat(lon, lat)
        if rel is not None:
            lines.append(f"基準点からの相対座標: X={rel[0]:.2f}m  Y={rel[1]:.2f}m")
        else:
            lines.append("基準点は未設定です（Oキーで設定モードへ）")

        if self.mode == "set_origin":
            lines.append(">>> 基準点設定モード: 地図をクリックしてください (Escでキャンセル)")
        elif self.mode == "add_point":
            lines.append(">>> 地点追加モード: 地図をクリックしてください (Escでキャンセル)")

        lines.append(f"保存済み地点数: {len(self.points)}  [O]基準点設定 [A]地点追加 [S]保存 [Esc]キャンセル")
        lines.append(self.status_message)

        y = 8
        for line in lines:
            surf = self.font_small.render(line, True, (0, 0, 0))
            bg = pygame.Surface((surf.get_width() + 8, surf.get_height() + 2))
            bg.fill((255, 255, 255))
            bg.set_alpha(210)
            self.screen.blit(bg, (6, y - 1))
            self.screen.blit(surf, (10, y))
            y += surf.get_height() + 4

        if self.text_input_active:
            self.draw_text_input_box()

    def draw_text_input_box(self):
        box_w, box_h = 420, 90
        box_x = (self.width - box_w) // 2
        box_y = (self.height - box_h) // 2
        pygame.draw.rect(self.screen, (255, 255, 255), (box_x, box_y, box_w, box_h))
        pygame.draw.rect(self.screen, (0, 0, 0), (box_x, box_y, box_w, box_h), 2)

        prompt = self.font.render("地点の名前を入力して Enter (Escでキャンセル):", True, (0, 0, 0))
        self.screen.blit(prompt, (box_x + 10, box_y + 8))

        input_text = self.font.render(self.text_input_buffer + "|", True, (0, 0, 200))
        self.screen.blit(input_text, (box_x + 10, box_y + 40))

    # --------------------------------------------------------
    # イベント処理
    # --------------------------------------------------------
    def handle_click(self, pos):
        """クリック（ドラッグではない）確定時の処理"""
        lon, lat = self.screen_to_lonlat(*pos)

        if self.mode == "set_origin":
            self.origin_meters = lonlat_to_meters(lon, lat)
            self.origin_lonlat = (lon, lat)
            self.status_message = "基準点(0,0)を設定しました"
            self.mode = None
            return

        if self.mode == "add_point":
            if self.origin_meters is None:
                self.status_message = "先に基準点を設定してください (Oキー)"
                self.mode = None
                return
            rel = self.relative_meters_from_lonlat(lon, lat)
            self.pending_point_meters = rel
            self.text_input_active = True
            self.text_input_buffer = ""
            self.mode = None
            return

    def confirm_text_input(self):
        name = self.text_input_buffer.strip()
        if name and self.pending_point_meters is not None:
            self.points[name] = self.pending_point_meters
            self.save_points()
            self.status_message = f'"{name}": ({self.pending_point_meters[0]:.2f}, ' \
                                   f'{self.pending_point_meters[1]:.2f}) を保存しました'
        else:
            self.status_message = "名前が空のため保存をキャンセルしました"
        self.text_input_active = False
        self.text_input_buffer = ""
        self.pending_point_meters = None

    def cancel_text_input(self):
        self.text_input_active = False
        self.text_input_buffer = ""
        self.pending_point_meters = None
        self.status_message = "入力をキャンセルしました"

    def process_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            elif event.type == pygame.VIDEORESIZE:
                self.width, self.height = event.w, event.h
                self.screen = pygame.display.set_mode(
                    (self.width, self.height), pygame.RESIZABLE
                )

            elif event.type == pygame.KEYDOWN:
                if self.text_input_active:
                    if event.key == pygame.K_RETURN:
                        self.confirm_text_input()
                    elif event.key == pygame.K_ESCAPE:
                        self.cancel_text_input()
                    elif event.key == pygame.K_BACKSPACE:
                        self.text_input_buffer = self.text_input_buffer[:-1]
                    else:
                        if event.unicode and event.unicode.isprintable():
                            self.text_input_buffer += event.unicode
                else:
                    if event.key == pygame.K_o:
                        self.mode = "set_origin"
                        self.status_message = "基準点設定モード: 地図をクリックしてください"
                    elif event.key == pygame.K_a:
                        self.mode = "add_point"
                        self.status_message = "地点追加モード: 地図をクリックしてください"
                    elif event.key == pygame.K_ESCAPE:
                        if self.mode is not None:
                            self.mode = None
                            self.status_message = "モードをキャンセルしました"
                    elif event.key == pygame.K_s:
                        self.save_points()
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        self.zoom_at((self.width / 2, self.height / 2), 1)
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        self.zoom_at((self.width / 2, self.height / 2), -1)

            elif event.type == pygame.MOUSEBUTTONDOWN:
                if self.text_input_active:
                    continue
                if event.button == 1:  # 左クリック開始
                    self.dragging = True
                    self.drag_last_pos = event.pos
                    self.drag_moved = False
                elif event.button == 4:  # ホイール上 = ズームイン
                    self.zoom_at(event.pos, 1)
                elif event.button == 5:  # ホイール下 = ズームアウト
                    self.zoom_at(event.pos, -1)

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    if self.dragging and not self.drag_moved:
                        # 動かずに離した -> クリックとして扱う
                        self.handle_click(event.pos)
                    self.dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.dragging and not self.text_input_active:
                    dx = event.pos[0] - self.drag_last_pos[0]
                    dy = event.pos[1] - self.drag_last_pos[1]
                    if abs(dx) > 2 or abs(dy) > 2:
                        self.drag_moved = True
                    self.pan_by_screen_delta(dx, dy)
                    self.drag_last_pos = event.pos

            elif event.type == pygame.MOUSEWHEEL:
                # 環境によっては MOUSEBUTTONDOWN(4/5) ではなく
                # こちらのイベントで飛んでくる場合がある
                if not self.text_input_active:
                    self.zoom_at(pygame.mouse.get_pos(), event.y)

    # --------------------------------------------------------
    # メインループ
    # --------------------------------------------------------
    def run(self):
        while self.running:
            self.process_events()
            self.draw_map()
            self.draw_markers()
            self.draw_ui()
            pygame.display.flip()
            self.clock.tick(60)
        pygame.quit()


def main():
    app = MapApp()
    app.run()


if __name__ == "__main__":
    main()
