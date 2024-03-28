import pygame
import numpy as np
import heapq

OBSTACLE_COLOR = (255, 0, 0)
SYMBOL_CARD_COLOR = (0, 255, 0)
CAR_COLOR = (0, 0, 255)

CAR_SIZE = 2

WHITE = (200, 200, 200)
BLACK = (0 , 0, 0)
GREEN = (31,244,151)


#CONSTANTS
WIDTH , HEIGHT = 1000, 1000
CELL_SIZE = 50
GRID_WIDTH, GRID_HEIGHT = WIDTH // CELL_SIZE, HEIGHT // CELL_SIZE
START_POS = (10,10)
END_POS = (GRID_WIDTH - 10, GRID_WIDTH - 10)

#Car Setting
car_x , car_y = 0, HEIGHT - CAR_SIZE * CELL_SIZE
velocity = 10
dx, dy = 0, 0

#Initialization
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("MDP Algorithm Visualizer")
clock = pygame.time.Clock()
screen.fill(WHITE)

#Grid data structure
grid = np.zeros((GRID_HEIGHT, GRID_WIDTH), dtype= int)
obstacles = [] # List of (x, y) coordinates for obstacles
symbolCards = [] # Dictionary of {(x, y): label} <- currently not using dictionary will implement in future

#

def main():
    global car_x, car_y, dx, dy

    #Event handling
    running = True
    while running :

        screen.fill(WHITE)
        drawGrid()
        drawObstacle()
        drawsymbolCard()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                pos = pygame.mouse.get_pos()
                cell_x, cell_y = pos[0] // CELL_SIZE , pos[1] // CELL_SIZE

                #Toggle obstacle at the given cell coordinate
                if event.key == pygame.K_q :
                    handleObstacleToggle(cell_x, cell_y)
                    continue
                #Toggle symbol card at the given cell coordinate
                elif event.key == pygame.K_e :
                    handleSymbolCardToggle(cell_x, cell_y)
                    continue

                #Car Movement logic
                elif event.key == pygame.K_UP:
                    dy = -velocity
                elif event.key == pygame.K_DOWN:
                    dy = velocity
                elif event.key == pygame.K_LEFT:
                    dx = -velocity
                elif event.key == pygame.K_RIGHT:
                    dx = velocity

            elif event.type == pygame.KEYUP:
                if event.key in (pygame.K_UP, pygame.K_DOWN) :
                    dy = 0
                elif event.key in (pygame.K_LEFT, pygame.K_RIGHT) :
                    dx = 0

        new_car_x = car_x + dx
        new_car_y = car_y + dy

        if(isSafe(new_car_x,new_car_y)) :
            print("Safe!")
        else :
            print("Is a collision !")

        drawCar()

        pygame.display.update()
        clock.tick(60)



def drawGrid() :
    for x in range(0, WIDTH, CELL_SIZE):
        for y in range(0, HEIGHT, CELL_SIZE):
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            pygame.draw.rect(screen, BLACK, rect, 1)

def drawObstacle() :
    for obstacle in obstacles:
        cell_x, cell_y = obstacle
        obstacle_rect = pygame.Rect(cell_x * CELL_SIZE, cell_y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, OBSTACLE_COLOR, obstacle_rect)

def drawCar():
    # Calculate the grid cell corresponding to the car's position
    car_grid_x = car_x // CELL_SIZE
    car_grid_y = car_y // CELL_SIZE

    # Draw the grid with the car's position lit up
    for x in range(0, WIDTH, CELL_SIZE):
        for y in range(0, HEIGHT, CELL_SIZE):
            rect = pygame.Rect(x, y, CELL_SIZE, CELL_SIZE)
            if (car_grid_x * CELL_SIZE <= x < (car_grid_x + CAR_SIZE) * CELL_SIZE and
                car_grid_y * CELL_SIZE <= y < (car_grid_y + CAR_SIZE) * CELL_SIZE):
                # Light up the grid cells that correspond to the 2x2 box of the car
                pygame.draw.rect(screen, CAR_COLOR, rect)
            else:
                pygame.draw.rect(screen, BLACK, rect, 1)

def drawsymbolCard() :
    for symbolCard in symbolCards:
        cell_x, cell_y = symbolCard #(x,y) tuple
        symbolCard_rect = pygame.Rect(cell_x * CELL_SIZE, cell_y * CELL_SIZE, CELL_SIZE, CELL_SIZE)
        pygame.draw.rect(screen, SYMBOL_CARD_COLOR , symbolCard_rect)

def handleObstacleToggle(cell_x, cell_y) :
    #if current location does not have an obstacle, we place an obstacle down
    if grid[cell_y, cell_x] == 0:
        grid[cell_y, cell_x] = 1
        if(cell_x,cell_y) not in symbolCards :
            obstacles.append((cell_x, cell_y))
    #clicking an existing obstacle will remove it from the grid
    elif grid[cell_y, cell_x] == 1:
        grid[cell_y, cell_x] = 0
        if (cell_x, cell_y) in obstacles :
            obstacles.remove((cell_x,cell_y))

def handleSymbolCardToggle(cell_x, cell_y) :
    #if current location does not have an symbolCard, we place an symbolCard down
    if grid[cell_y, cell_x] == 0:
        grid[cell_y, cell_x] = 1
        if(cell_x,cell_y) not in obstacles :
            symbolCards.append((cell_x, cell_y))
    #clicking an existing symbolCard will remove it from the grid
    elif grid[cell_y, cell_x] == 1:
        grid[cell_y, cell_x] = 0
        if (cell_x, cell_y) in symbolCards :
            symbolCards.remove((cell_x,cell_y))

def isSafe(new_car_x, new_car_y) :
    global car_x, car_y
    collision = False
    if(0 <= new_car_x <= WIDTH - CAR_SIZE * CELL_SIZE) and (0 <= new_car_y <= HEIGHT - CAR_SIZE * CELL_SIZE):
        for obstacle in obstacles :
            obstacle_x, obstacle_y = obstacle
            # ensures that if the new position of the car overlaps with any part of an obstacle (both horizontally and vertically)
            if(obstacle_x * CELL_SIZE <= new_car_x < (obstacle_x + CELL_SIZE) * CAR_SIZE and 
                (obstacle_y < new_car_y < obstacle_y + CELL_SIZE * CAR_SIZE)) :
                collision = True
                return False
                
        if not collision:      
            car_grid_x = int(new_car_x // CELL_SIZE)
            car_grid_y = int(new_car_y // CELL_SIZE)
            if 0 <= car_grid_x < GRID_WIDTH - CAR_SIZE + 1 and 0 <= car_grid_y < GRID_HEIGHT - CAR_SIZE + 1:
                for x in range(car_grid_x, car_grid_x + CAR_SIZE):
                    for y in range(car_grid_y, car_grid_y + CAR_SIZE):
                        if grid[y][x] == 1:  # Collision with obstacle in the grid
                            collision = True
                            break

                    if collision:
                        return False

        if not collision:
            car_x = new_car_x
            car_y = new_car_y
            return True
        
        else:
            return False
    else:
        return False

if __name__ == "__main__":
    main()