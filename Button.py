import pygame
pygame.init()

screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("dijkstra")

class Button:
    def __init__(self, location:tuple[int,int], image: str, text: str, image_size: tuple[int, int]) -> None:
        self.x = location[0]
        self.y = location[1]
        self.text = text
        self.image = pygame.transform.scale(pygame.image.load(image), image_size)
        self.clicked = False

    def draw(self, surface) -> bool:
        action = False
        pos = pygame.mouse.get_pos()
        rect = self.image.get_rect(topleft=(self.x, self.y))


        if rect.collidepoint(pos):
            if pygame.mouse.get_pressed()[0] == 1 and not self.clicked:
                print("clicked:", self.text)
                self.clicked = True
                action = True

        if pygame.mouse.get_pressed()[0] == 0:
            self.clicked = False


        font = pygame.font.SysFont("Arial", 36)
        text_surface = font.render(self.text, True, (0, 0, 0))

        surface.blit(self.image, (self.x, self.y))
        surface.blit(text_surface, (self.x + self.image.get_width(), self.y))

        return action

# b = Button((50, 50), "Tomo.png", "dijkstra", (100, 100))
# running = True
# while running:
#     screen.fill((30, 30, 30))  # Clear screen
#
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             running = False
#
#     b.draw(screen)
#
#     pygame.display.flip()
#
# pygame.quit()