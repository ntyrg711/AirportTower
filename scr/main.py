import math
import pygame

from settings import WIDTH, HEIGHT, CLICK_RADIUS
from graph import draw_graph
from movement import get_destination_buttons, build_preview_legs
from aircraft import Aircraft
from camera import Camera

pygame.init()

screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 24)

camera = Camera()

aircrafts = [
    Aircraft("ANA123", "A-E"),
    Aircraft("JAL456", "E-H"),
    Aircraft("SKY789", "B-M"),
]

selected = None
running = True

# 右クリックドラッグでのパン(画面移動)用の状態
panning = False
last_mouse_pos = (0, 0)

while running:
    buttons = get_destination_buttons(selected)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEWHEEL:
            # スクロールで、マウス位置を中心に拡大縮小する
            factor = 1.1 if event.y > 0 else (1 / 1.1)
            camera.zoom_at(pygame.mouse.get_pos(), WIDTH, HEIGHT, factor)

        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = event.pos

            if event.button == 3:
                # 右クリックドラッグでパン開始(左クリックは選択・移動指示に使うため分ける)
                panning = True
                last_mouse_pos = (mx, my)

            elif event.button == 1:
                handled = False

                for kind, rect, label, route in buttons:
                    if rect.collidepoint(mx, my):
                        if kind == "route":
                            # TaxiWayを選ぶ = 実際には動かさずキューに追加するだけ
                            selected.add_to_plan(route)
                        elif kind == "hold_short":
                            # TaxiWayを選ぶが、今乗っているTaxiWayの終点
                            # までは乗らず、次の交差点の手前で止まる経路
                            # (hold_short_route)をキューに追加し、そのまま
                            # 選択終了も自動で行う(hold short自体が
                            # 「ここまでの計画で確定・実行開始」の意思表示
                            # のため)。停止位置が常に交差点の手前になる
                            # のはどの経路でも同じなので、ここでは経路
                            # (どこで止めるか)だけが通常の"route"と違う。
                            selected.add_to_plan(route)
                            selected.confirm_plan()
                        elif kind == "finish":
                            # 選択終了 = キューに積んだ経路をまとめて移動開始
                            # (移動開始のタイミングでのみ回転する)
                            selected.confirm_plan()
                        handled = True
                        break

                if not handled:
                    clicked_aircraft = None
                    for aircraft in aircrafts:
                        sx, sy = camera.world_to_screen(aircraft.x, aircraft.y, WIDTH, HEIGHT)
                        click_radius = CLICK_RADIUS * camera.zoom
                        if math.hypot(mx - sx, my - sy) < click_radius:
                            clicked_aircraft = aircraft
                            break

                    if clicked_aircraft is not None:
                        if selected is not None and selected is not clicked_aircraft:
                            selected.cancel_plan()
                        selected = clicked_aircraft
                        handled = True

                if not handled:
                    if selected is not None:
                        selected.cancel_plan()
                    selected = None

        elif event.type == pygame.MOUSEBUTTONUP:
            if event.button == 3:
                panning = False

        elif event.type == pygame.MOUSEMOTION:
            if panning:
                mx, my = event.pos
                dx = mx - last_mouse_pos[0]
                dy = my - last_mouse_pos[1]
                camera.pan(dx, dy)
                last_mouse_pos = (mx, my)

    for aircraft in aircrafts:
        aircraft.update(aircrafts)

    screen.fill((30, 30, 30))
    planned_legs = build_preview_legs(selected)
    draw_graph(screen, font, camera, planned_legs=planned_legs)

    for kind, rect, label, route in buttons:
        if kind == "finish":
            color = (60, 130, 70)
        elif kind == "hold_short":
            color = (150, 100, 30)
        else:
            color = (80, 80, 80)
        pygame.draw.rect(screen, color, rect)
        txt = font.render(label, True, (255, 255, 255))
        screen.blit(txt, (rect.x + 10, rect.y + 5))

    for aircraft in aircrafts:
        aircraft.draw(screen, font, camera, aircraft is selected)

    pygame.display.flip()
    clock.tick(60)

pygame.quit()