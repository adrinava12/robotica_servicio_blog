# Práctica 2: Rescate de personas

## Introducción

En esta práctica se pide programar un dron para la **localización de supervivientes en alta mar**.
Para la realización de esta práctica, se ha creado una **máquina de estados** con los siguientes estados:

* Aproximación a la zona de búsqueda
* Búsqueda
* Vuelta a la lancha

Para detectar a los supervivientes durante el estado de búsqueda, se ha usado el detector de caras **HaarCascade de OpenCV**.

## Estado 1: Aproximación a la zona de búsqueda

Este estado es muy simple: consiste en **despegar de la lancha** e ir a la zona de búsqueda marcada.
Se cambiará de estado una vez **se haya llegado a la zona de búsqueda**, y para comprobarlo se realiza de la siguiente manera:

```python
pose = HAL.get_position()
x, y, z = pose
x_obj, y_obj, z_obj = SURVIVORS_EST_POS
distancia = math.sqrt((x_obj-x)**2 + (y_obj-y)**2)

# Ya se ha llegado, pasar a estado de búsqueda
if distancia < 0.25:
    return SEARCH

# Todavía no se ha llegado, seguir en el mismo estado
else:
    return GO2SEARCH_ZONE
```

## Estado 2: Búsqueda de supervivientes

Este estado se alcanza **una vez se haya llegado a la zona de búsqueda**.
El proceso de búsqueda se realiza de la siguiente manera:

### Creación de puntos en espiral

Antes de ejecutar la máquina de estados, se crea una serie de puntos en forma de **espiral** que el dron seguirá para realizar la búsqueda:

```python
def create_path(x_centro, y_centro):
    path = []
    # Distancia entre celdas, calculada con el teorema del seno
    paso = (ALTURA_BUSQUEDA * math.sin(math.radians(90))) / math.sin(math.radians(180 - 90 - (ANGULO_CAMARA / 2)))

    x, y = x_centro, y_centro
    
    # Direcciones: derecha, abajo, izquierda, arriba
    dx = [1, 0, -1, 0]
    dy = [0, -1, 0, 1]
    
    longitud = 1  # pasos a mover en cada dirección

    # Generar puntos del path hasta completar la espiral
    while len(path) < (2*N_VUELTAS + 1)**2:
        for i in range(4):
            for _ in range(longitud):
                x += dx[i] * paso
                y += dy[i] * paso
                path.append((x, y))
            # Incrementar la longitud cada dos direcciones
            if i % 2 == 1:
                longitud += 1
    return path
```

### Recorrido de los puntos de la espiral

Una vez en este estado, el dron seguirá la espiral de puntos creada previamente.
Se calculará **el ángulo del dron** necesario para llegar a cada punto y se comprobará la distancia para saber cuándo pasar al siguiente punto:

```python
# Calcular ángulo del dron
x1, y1, _ = HAL.get_position()  # Punto actual
x2, y2 = path_search[i]          # Punto siguiente
dx = x2 - x1
dy = y2 - y1
angulo = math.atan2(dy, dx)

# Comandar posición
HAL.set_cmd_pos(x2, y2, ALTURA_BUSQUEDA, angulo)

# Comprobar si se ha llegado
while distancia > 0.5:
    x1, y1, _ = HAL.get_position()
    distancia = math.sqrt((x2-x1)**2 + (y2-y1)**2)
```

### Detección de caras

Durante la comprobación de distancia al siguiente objetivo de la espiral, se toman fotos **continuamente con la cámara ventral del dron**.
Cada foto se rota 5° hasta completar una vuelta y se pasa al **detector Haar Cascade**:

```python
angles = list(range(0, 360, 5))  # Girar cada 5 grados
pos_faces = []

(h, w) = frame.shape[:2]
center = (w/2, h/2)

for angle in angles:
    # Rotar imagen
    M = cv.getRotationMatrix2D(center, angle, 1.0)
    rotated = cv.warpAffine(frame, M, (w, h), flags=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)

    # Detectar caras en la imagen rotada
    frame_gray = cv.cvtColor(rotated, cv.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(frame_gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
```

### Transformación de coordenadas de la imagen a Gazebo

Una vez detectada la cara, se transforman las coordenadas de **píxeles de la imagen** al **marco original**:

```python
# Centro de la cara en la imagen rotada
cx_rot = x + fw/2
cy_rot = y + fh/2

# Transformar coordenadas de vuelta al marco original
M_inv = cv.invertAffineTransform(M)
pt_rot = np.array([cx_rot, cy_rot, 1])
cx_orig, cy_orig = M_inv @ pt_rot
```

Luego, se transforman estas coordenadas al **mundo de Gazebo**:

```python
# Convertir de píxeles a coordenadas del mundo
dx_pix = (face[0] - center[0])
dy_pix = (face[1] - center[1])

# Ángulo que cubre un píxel
angle_per_pixel_x = math.radians(ANGULO_CAMARA) / (center[0] * 2)
angle_per_pixel_y = math.radians(ANGULO_CAMARA) / (center[1] * 2)

# Ángulo de la detección respecto al eje óptico
theta_x = dx_pix * angle_per_pixel_x
theta_y = dy_pix * angle_per_pixel_y

# Desplazamiento en el marco de la cámara (en metros)
dx_cam = ALTURA_BUSQUEDA * math.tan(theta_x)
dy_cam = ALTURA_BUSQUEDA * math.tan(theta_y)

# Orientación del dron (yaw)
_, _, yaw = HAL.get_orientation()

# Rotar desde cámara → mundo (X derecha, Y abajo)
dx_world = dx_cam * math.cos(yaw) - dy_cam * math.sin(yaw)
dy_world = dx_cam * math.sin(yaw) + dy_cam * math.cos(yaw)

pose = HAL.get_position()
X_face = pose[0] + dx_world
Y_face = pose[1] + dy_world

return (X_face, Y_face)
```

### Filtrado de caras cercanas

Una vez tenemos las coordenadas de todas las caras detectadas en Gazebo, se eliminan **detecciones cercanas** que correspondan a la misma persona:

```python
def add_unique_faces(new_faces, global_faces):
    for (xf, yf) in new_faces:
        if not any(math.hypot(xf - gx, yf - gy) < 3.0 for (gx, gy) in global_faces):
            global_faces.append((xf, yf))
    return global_faces
```

### Conversión de coordenadas de Gazebo a GPS

Finalmente, se convierten las coordenadas de Gazebo a **GPS**, entendibles para los equipos de rescate:

```python
def world_to_gps(x, y):
    # Metros por grado
    m_per_deg_lat = 111_320
    m_per_deg_lon = 111_320 * math.cos(math.radians(LAT0))
    
    delta_lat = -y / m_per_deg_lat
    delta_lon = x / m_per_deg_lon
    
    lat = LAT0 + delta_lat
    lon = LON0 + delta_lon
    return (lat, lon)
```

Este estado de búsqueda finaliza **cuando se alcanza un nivel crítico de batería** o se ha recorrido toda la espiral marcada.

## Estado 3: Vuelta a la lancha

Este estado se alcanza **cuando la batería del dron baja del nivel crítico**.
El dron deja la búsqueda y regresa a la lancha. Al llegar, **aterriza automáticamente**.

## Vídeo

[Ver demostración búsqueda de supervivientes](https://drive.google.com/file/d/1wdCD7wgKnwyFG_e4t5bLTjfIXgq9u-cP/view?usp=sharing)
