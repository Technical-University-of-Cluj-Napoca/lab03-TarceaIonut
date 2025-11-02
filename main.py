from Button import Button

import pygame

from utils import *
from grid import Grid

pygame.init()

def get_button_coordinates(nr_buttons: int) -> tuple[list[tuple[int, int]], tuple[int,int]] | None:
    spare_with = WIDTH - WIDTH_GRID

    button_width_max = (spare_with * 0.6)
    button_height_max = HEIGHT * 0.8 / (nr_buttons * 1.1)

    if button_width_max >= 3 * button_height_max:
        real_button_height = button_height_max
        real_button_width = button_height_max * 2
    else:
        real_button_width = button_width_max
        real_button_height = button_width_max / 2

    button_coordinates_w:int = int(WIDTH_GRID + ((WIDTH - WIDTH_GRID) - real_button_width * 1.5) / 2)
    button_coordinates_h:int = int((HEIGHT - (real_button_height * nr_buttons * 1.1)) / 2)

    l: list[tuple[int, int]] = []

    for i in range(0, nr_buttons):
        l.append((button_coordinates_w, button_coordinates_h))
        button_coordinates_h += int(real_button_height * 1.1)

    return l, (int(real_button_width), int(real_button_height))

from searching_algorithms import *

if __name__ == "__main__":
    # setting up how big will be the display window
    WIN = pygame.display.set_mode((WIDTH, HEIGHT))

    # set a caption for the window
    pygame.display.set_caption("Path Visualizing Algorithm")

    ROWS_CURRENT = ROWS
    COLS_CURRENT = COLS
    grid = Grid(WIN, ROWS_CURRENT, COLS_CURRENT, WIDTH_GRID, HEIGHT_GRID)

    start = None
    end = None

    # flags for running the main loop
    run = True
    started = False

    algorithm:str = "astar"
    sim_stopped = False


    l, (b_h, b_w) = get_button_coordinates(13)

    print(l)
    print(b_h, b_w)

    button_bfs = Button(l[0], "Kagura.png", "bfs", (b_h, b_w))
    button_dfs = Button(l[1], "Yukary.png", "dfs", (b_h, b_w))
    button_dls = Button(l[2], "Yomi.png", "dls", (b_h, b_w))
    button_astar = Button(l[3], "Tomo.png", "astar", (b_h, b_w))
    button_ucs = Button(l[4], "Sakaki.png", "ucs", (b_h, b_w))
    button_dijkstra = Button(l[5], "Chiyo.png", "dijkstra", (b_h, b_w))
    button_iddfs = Button(l[6], "Osaka.png", "iddfs", (b_h, b_w))
    button_ida = Button(l[7], "Chihiro.png", "ida", (b_h, b_w))
    button_stop_sim = Button(l[8], "Dio.png", "stop_sim", (b_h, b_w))
    button_reset_sim = Button(l[9], "Made in heaven.png", "reset_sim", (b_h, b_w))
    button_reset_board = Button(l[10], "bites the dust.png", "reset_board", (b_h, b_w))
    button_more_cels = Button(l[11], "Rei.png", "more_cels", (b_h, b_w))
    button_less_cels = Button(l[12], "Tanos.png", "less_cels", (b_h, b_w))


    def draw_window_handler() -> bool:
        comm = draw_window()
        if comm == COMMAND["STOP"]:
            while True:
                comm = draw_window()
                if comm != COMMAND["NOTHING"]:
                    break
        return not comm == COMMAND["EXIT"]


    def draw_window() -> COMMAND:
        pygame.event.pump()
        global algorithm
        global grid
        global start
        global end
        global ROWS_CURRENT
        global COLS_CURRENT
        grid.draw()
        if button_bfs.draw(WIN):
            algorithm = button_bfs.text
        if button_dfs.draw(WIN):
            algorithm = button_dfs.text
        if button_dls.draw(WIN):
            algorithm = button_dls.text
        if button_astar.draw(WIN):
            algorithm = button_astar.text
        if button_ucs.draw(WIN):
            algorithm = button_ucs.text
        if button_dijkstra.draw(WIN):
            algorithm = button_dijkstra.text
        if button_iddfs.draw(WIN):
            algorithm = button_iddfs.text
        if button_ida.draw(WIN):
            algorithm = button_ida.text

        if button_stop_sim.draw(WIN):
            return COMMAND["STOP"]

        if button_reset_sim.draw(WIN):
            return COMMAND["EXIT"]

        if button_reset_board.draw(WIN):
            grid = Grid(WIN, ROWS_CURRENT, COLS_CURRENT, WIDTH_GRID, HEIGHT_GRID)
            start = None
            end = None
            return COMMAND["EXIT"]


        if button_more_cels.draw(WIN):
            ROWS_CURRENT = ROWS_CURRENT * 2
            COLS_CURRENT = COLS_CURRENT * 2
            grid = Grid(WIN, ROWS_CURRENT, COLS_CURRENT, WIDTH_GRID, HEIGHT_GRID)
            start = None
            end = None
            return COMMAND["EXIT"]

        if button_less_cels.draw(WIN):
            ROWS_CURRENT = ROWS_CURRENT // 2
            COLS_CURRENT = COLS_CURRENT // 2
            grid = Grid(WIN, ROWS_CURRENT, COLS_CURRENT, WIDTH_GRID, HEIGHT_GRID)
            start = None
            end = None
            return COMMAND["EXIT"]

        #pygame.display.flip()
        pygame.display.update()

        return COMMAND["NOTHING"]



    while run:
        #grid.draw()  # draw the grid and its spots
        draw_window()



        for event in pygame.event.get():
            # verify what events happened
            if event.type == pygame.QUIT:
                run = False

            if started:
                # do not allow any other interaction if the algorithm has started
                continue  # ignore other events if algorithm started

            if pygame.mouse.get_pressed()[0]:  # LEFT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)

                if row >= ROWS_CURRENT or row < 0 or col >= COLS_CURRENT or col < 0:
                    continue  # ignore clicks outside the grid

                spot = grid.grid[row][col]
                if not start and spot != end:
                    start = spot
                    start.make_start()
                elif not end and spot != start:
                    end = spot
                    end.make_end()
                elif spot != end and spot != start:
                    spot.make_barrier()

            elif pygame.mouse.get_pressed()[2]:  # RIGHT CLICK
                pos = pygame.mouse.get_pos()
                row, col = grid.get_clicked_pos(pos)
                spot = grid.grid[row][col]
                spot.reset()

                if spot == start:
                    start = None
                elif spot == end:
                    end = None

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE and not started:
                    # run the algorithm
                    for row in grid.grid:
                        for spot in row:
                            spot.update_neighbors(grid.grid)
                    # here you can call the algorithms

                    print(algorithm)



                    if algorithm == "astar":
                        astar(draw_window_handler, grid, start, end)
                    if algorithm == "bfs":
                        bfs(draw_window_handler, grid, start, end)
                    if algorithm == "dfs":
                        dfs(draw_window_handler, grid, start, end)
                    if algorithm == "dls":
                        dls(draw_window_handler, grid, start, end)
                    if algorithm == "ucs":
                        ucs(draw_window_handler, grid, start, end)
                    if algorithm == "dijkstra":
                        dijkstra(draw_window_handler, grid, start, end)
                    if algorithm == "iddfs":
                        iddfs(draw_window_handler, grid, start, end)
                    if algorithm == "ida":
                        ida(draw_window_handler, grid, start, end)
                    started = False

                    # grid = Grid(WIN, ROWS, COLS, WIDTH_GRID, HEIGHT_GRID)
                    # start = None
                    # end = None

                    #grid = Grid(WIN, ROWS * 2, COLS * 2, WIDTH, HEIGHT)

                if event.key == pygame.K_c:
                    print("Clearing the grid...")
                    start = None
                    end = None
                    grid.reset()
    pygame.quit()


