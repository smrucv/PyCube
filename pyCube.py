import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import math, random
import kociemba  # Asegúrate de tenerlo instalado: pip install kociemba

### Funciones para matrices 4x4 ###
def rotation_matrix(axis, angle):
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1 - c
    x, y, z = axis
    return np.array([[t*x*x + c,    t*x*y - s*z,  t*x*z + s*y, 0],
                     [t*x*y + s*z,  t*y*y + c,    t*y*z - s*x, 0],
                     [t*x*z - s*y,  t*y*z + s*x,  t*z*z + c,   0],
                     [0,            0,            0,           1]], dtype=float)

def translation_matrix(tx, ty, tz):
    return np.array([[1, 0, 0, tx],
                     [0, 1, 0, ty],
                     [0, 0, 1, tz],
                     [0, 0, 0, 1]], dtype=float)

def rotate_vector(vec, axis, angle):
    """
    Rota un vector 3D 'vec' (numpy array de 3 elementos) respecto al eje 'axis' por 'angle' (radianes),
    usando la fórmula de Rodrigues.
    """
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return vec * cos_a + np.cross(axis, vec) * sin_a + axis * (np.dot(axis, vec)) * (1 - cos_a)

### Rutina para actualizar el estado facelet para la cara Up (movimiento U)
def update_face_state_U(face_state, move):
    """
    Actualiza la cadena facelet para un movimiento Up.
    La cadena tiene 54 caracteres en el orden: U (0–8), R (9–17), F (18–26), D (27–35), L (36–44), B (45–53).
    Usamos la convención:
      W -> U, R -> R, G -> F, Y -> D, O -> L, B -> B.
    Para el movimiento 'w' (giro horario) se aplica la siguiente permutación:
      En la cara U (índices 0..8): 
          [0,1,2,3,4,5,6,7,8] -> [6,3,0,7,4,1,8,5,2]
      Y se intercambian las filas superiores de F, R, B y L en el siguiente ciclo:
          F[0..2] -> R[0..2] -> B[0..2] -> L[0..2] -> F[0..2]
    Para 'W' (anticlockwise) se aplica la permutación inversa.
    """
    state = list(face_state)
    if move == 'w':
        newU = [state[6], state[3], state[0],
                state[7], state[4], state[1],
                state[8], state[5], state[2]]
        state[0:9] = newU
        temp = state[18:21]  # F
        state[18:21] = state[36:39]  # L -> F
        state[36:39] = state[45:48]  # B -> L
        state[45:48] = state[9:12]   # R -> B
        state[9:12]  = temp          # F -> R
    elif move == 'W':
        newU = [state[2], state[5], state[8],
                state[1], state[4], state[7],
                state[0], state[3], state[6]]
        state[0:9] = newU
        temp = state[9:12]  # R
        state[9:12]  = state[45:48]  # B -> R
        state[45:48] = state[36:39]  # L -> B
        state[36:39] = state[18:21]  # F -> L
        state[18:21] = temp          # R -> F
    return "".join(state)

### Clase Cubie ###
class Cubie:
    def __init__(self, logical_pos, spacing):
        # logical_pos: vector (x,y,z) con valores en {-1,0,1}
        self.logical_pos = np.array(logical_pos, dtype=int)
        self.orientation = np.identity(4, dtype=float)
        self.transform = translation_matrix(
            logical_pos[0] * spacing,
            logical_pos[1] * spacing,
            logical_pos[2] * spacing
        ).dot(self.orientation)
        self.colors = {}
        if logical_pos[1] == 1:
            self.colors['W'] = (1, 1, 1)       # Up: blanco
        if logical_pos[1] == -1:
            self.colors['Y'] = (1, 1, 0)       # Down: amarillo
        if logical_pos[2] == 1:
            self.colors['G'] = (0, 1, 0)       # Front: verde
        if logical_pos[2] == -1:
            self.colors['B'] = (0, 0, 1)       # Back: azul
        if logical_pos[0] == 1:
            self.colors['R'] = (1, 0, 0)       # Right: rojo
        if logical_pos[0] == -1:
            self.colors['O'] = (1, 0.5, 0)     # Left: naranja

### Clase RubikCube ###
class RubikCube:
    def __init__(self, spacing=1.0):
        self.spacing = spacing
        self.cubies = []
        self.reset()  # Crea el cubo en estado resuelto
    
    def reset(self):
        """Reinicia el cubo al estado resuelto."""
        self.cubies = []
        for x in [-1, 0, 1]:
            for y in [-1, 0, 1]:
                for z in [-1, 0, 1]:
                    if x == 0 and y == 0 and z == 0:
                        continue
                    self.cubies.append(Cubie((x, y, z), self.spacing))
        # Estado facelet resuelto: U, R, F, D, L, B
        self.face_state = "WWWWWWWWW" + "RRRRRRRRR" + "GGGGGGGGG" + "YYYYYYYYY" + "OOOOOOOOO" + "BBBBBBBBB"
    
    def apply_move(self, move, steps=20, tr=15):
        """
        Aplica un movimiento (p.ej. "w" o "W") de 90° a la cara correspondiente.
        Minúscula = giro horario (+90°); mayúscula = giro antihorario (-90°).
        Se anima la rotación sobre el centro de la cara y se actualiza la orientación acumulada.
        """
        letter = move[0].upper()
        angle_total = math.pi/2 if move[0].islower() else -math.pi/2
        
        # Definir eje, condición y centro de rotación según la letra.
        # (Aquí se implementa para cada cara según la convención de colores.)
        if letter == 'W':  # Up
            axis = (0, 1, 0)
            condition = lambda c: c.logical_pos[1] == 1
            center = np.array([0, self.spacing, 0], dtype=float)
        elif letter == 'Y':  # Down
            axis = (0, 1, 0)
            condition = lambda c: c.logical_pos[1] == -1
            center = np.array([0, -self.spacing, 0], dtype=float)
        elif letter == 'G':  # Front
            axis = (0, 0, 1)
            condition = lambda c: c.logical_pos[2] == 1
            center = np.array([0, 0, self.spacing], dtype=float)
        elif letter == 'B':  # Back
            axis = (0, 0, 1)
            condition = lambda c: c.logical_pos[2] == -1
            center = np.array([0, 0, -self.spacing], dtype=float)
        elif letter == 'R':  # Right
            axis = (1, 0, 0)
            condition = lambda c: c.logical_pos[0] == 1
            center = np.array([self.spacing, 0, 0], dtype=float)
        elif letter == 'O':  # Left
            axis = (1, 0, 0)
            condition = lambda c: c.logical_pos[0] == -1
            center = np.array([-self.spacing, 0, 0], dtype=float)
        else:
            return
        
        affected = [c for c in self.cubies if condition(c)]
        delta_angle = angle_total / steps
        rot_total = rotation_matrix(axis, angle_total)
        for i in range(steps):
            rot = rotation_matrix(axis, delta_angle)
            T1 = translation_matrix(-center[0], -center[1], -center[2])
            T2 = translation_matrix(center[0], center[1], center[2])
            for cubie in affected:
                cubie.transform = T2.dot(rot).dot(T1).dot(cubie.transform)
            draw_scene(self)
            pygame.display.flip()
            pygame.time.wait(tr)
        for cubie in affected:
            cubie.orientation = rot_total.dot(cubie.orientation)
            new_logical = rotate_vector(cubie.logical_pos.astype(float), np.array(axis, dtype=float), angle_total)
            cubie.logical_pos = np.rint(new_logical).astype(int)
            pos = cubie.logical_pos.astype(float) * self.spacing
            cubie.transform = translation_matrix(pos[0], pos[1], pos[2]).dot(cubie.orientation)
        
        # Actualizar el estado facelet interno para movimientos de Up (W).
        # (Para un solver completo se deben implementar las permutaciones para todas las caras.)
        if letter == 'W':
            self.face_state = update_face_state_U(self.face_state, move)
        # Si se aplican otros movimientos, deberás actualizar self.face_state con la permutación correspondiente.
    
    def scramble(self):
        """Aplica 30 movimientos aleatorios para mezclar el cubo."""
        valid_moves = ['w','W','y','Y','g','G','b','B','r','R','o','O']
        moves_applied = []
        for _ in range(30):
            move = random.choice(valid_moves)
            moves_applied.append(move)
            self.apply_move(move, tr=2)
        print("Scramble moves:", " ".join(moves_applied))
    
    def get_facelet_state(self):
        """
        Devuelve el estado facelet interno (cadena de 54 caracteres).
        """
        return self.face_state
    
    def solve(self):
        """
        Usa Kociemba para resolver el cubo a partir de su estado facelet.
        Devuelve la secuencia de movimientos (cadena) o un mensaje de error.
        """
        state = self.get_facelet_state()
        print("Estado facelet:", state)
        print(len(state))
        try:
            solution = kociemba.solve(state)
        except Exception as e:
            solution = "Error: " + str(e)
        return solution

### Renderizado con PyOpenGL ###
def draw_cubie(cubie):
    s = 0.9
    vertices = [
        np.array([-s/2, -s/2, -s/2, 1]),
        np.array([ s/2, -s/2, -s/2, 1]),
        np.array([ s/2,  s/2, -s/2, 1]),
        np.array([-s/2,  s/2, -s/2, 1]),
        np.array([-s/2, -s/2,  s/2, 1]),
        np.array([ s/2, -s/2,  s/2, 1]),
        np.array([ s/2,  s/2,  s/2, 1]),
        np.array([-s/2,  s/2,  s/2, 1])
    ]
    transformed = []
    for v in vertices:
        tv = cubie.transform.dot(v)
        transformed.append(tv[:3])
    faces = [
        ([4,5,6,7], 'G'),
        ([1,0,3,2], 'B'),
        ([5,1,2,6], 'R'),
        ([0,4,7,3], 'O'),
        ([3,7,6,2], 'W'),
        ([0,1,5,4], 'Y')
    ]
    for inds, key in faces:
        if key in cubie.colors:
            col = cubie.colors[key]
        else:
            col = (0.1, 0.1, 0.1)
        glColor3f(*col)
        glBegin(GL_QUADS)
        for i in inds:
            glVertex3f(*transformed[i])
        glEnd()

def draw_scene(cube):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    global camera_theta, camera_phi, camera_distance
    cam_x = camera_distance * math.sin(camera_phi) * math.cos(camera_theta)
    cam_y = camera_distance * math.cos(camera_phi)
    cam_z = camera_distance * math.sin(camera_phi) * math.sin(camera_theta)
    gluLookAt(cam_x, cam_y, cam_z, 0, 0, 0, 0, 1, 0)
    for cubie in cube.cubies:
        draw_cubie(cubie)

def init_opengl(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, (width/height), 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glEnable(GL_DEPTH_TEST)

### Parámetros de cámara (movimiento continuo)
camera_theta = math.radians(45)
camera_phi = math.radians(45)
camera_distance = 15

### Función principal ###
def main():
    pygame.init()
    display_width = 800
    display_height = 600
    screen = pygame.display.set_mode((display_width, display_height), DOUBLEBUF | OPENGL)
    init_opengl(display_width, display_height)
    
    cube = RubikCube(spacing=1.0)  # Cubies casi pegados
    clock = pygame.time.Clock()
    running = True
    global camera_theta, camera_phi, camera_distance
    
    while running:
        dt = clock.tick(60) / 1000.0  # tiempo en segundos
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            elif event.type == KEYDOWN:
                key = event.unicode
                if key in ['w','W','y','Y','g','G','b','B','r','R','o','O']:
                    cube.apply_move(key)
                elif key in ['s','S']:
                    cube.scramble()
                elif key == '0':
                    cube.reset()
                elif key in ['v','V']:
                    solution = cube.solve()
                    print("Solución:", solution)
                    if not solution.startswith("Error"):
                        # Aplicar la solución (aquí se necesita mapear la notación de Kociemba a la de nuestro simulador)
                        mapping = {'U':'w','R':'r','F':'g','D':'y','L':'o','B':'B'}
                        for move in solution.split():
                            sim_move = mapping.get(move[0], move[0])
                            if len(move) > 1:
                                if move[1] == "'":
                                    sim_move = sim_move.upper()
                                elif move[1] == '2':
                                    cube.apply_move(sim_move)
                                    cube.apply_move(sim_move)
                                    continue
                            cube.apply_move(sim_move)
                elif key == '+':
                    camera_distance = max(1, camera_distance - 1)
                elif key == '-':
                    camera_distance += 1
        # Movimiento continuo de cámara
        keys = pygame.key.get_pressed()
        camera_speed = math.radians(60)
        if keys[K_LEFT]:
            camera_theta -= camera_speed * dt
        if keys[K_RIGHT]:
            camera_theta += camera_speed * dt
        if keys[K_UP]:
            camera_phi = max(math.radians(1), camera_phi - camera_speed * dt)
        if keys[K_DOWN]:
            camera_phi = min(math.radians(179), camera_phi + camera_speed * dt)
        
        draw_scene(cube)
        pygame.display.flip()
    pygame.quit()

if __name__ == "__main__":
    main()
