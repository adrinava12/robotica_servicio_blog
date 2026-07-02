# Laser Mapping

## Resumen

En esta práctica se pide programar un robot equipado con un sensor láser y autolocalización para que construya un mapa de ocupación probabilístico en forma de rejilla de la nave industrial en la que se encuentra. Para ello hay que implementar dos partes: un algoritmo de exploración que haga deambular al robot por el entorno de manera autónoma, y un algoritmo de construcción del gridmap probabilístico usando la regla de Bayes mediante log-odds.

---

## Navegación por Tiempo Aleatorio Controlado

La estrategia de navegación se basa en avanzar en línea recta de forma constante (`V_CRUISE = 0.4`) hasta que los sectores del láser detectan que la zona frontal está bloqueada a menos de `0.50` metros.

Para la lectura del entorno, las muestras del láser se dividen en tres sectores estáticos definidos por sus índices:
```python
FRONT_IDX = list(range(0, 40)) + list(range(320, 360))
LEFT_IDX  = list(range(40, 180))
RIGHT_IDX = list(range(180, 320))
```

Cuando la distancia mínima del sector frontal (`front`) baja del umbral de seguridad, el robot se detiene e inicia una maniobra de rotación sobre su propio eje (`W_TURN = 0.7`). El sentido del giro (izquierda o derecha) se decide de forma inteligente comparando qué sector lateral está más despejado:

```python
# Elige el lado que esté más vacío para esquivar el obstáculo
if left > right:
    turn_dir = 1   # Izquierda
elif right > left:
    turn_dir = -1  # Derecha
else:
    turn_dir = random.choice([1, -1])

# Rango aleatorio en radianes equivalente a un giro de entre 70 y 110 grados
random_turn_rad = random.uniform(math.radians(70), math.radians(110))

# Tiempo necesario basado en la velocidad angular constante
duration = random_turn_rad / W_TURN
turn_end_time = time.time() + duration
is_turning = True

return 0.0, turn_dir * W_TURN
```

Mientras la bandera `is_turning` permanezca activa, el robot ignora por completo las lecturas actuales del láser y se mantiene rotando en el sitio. La transformación de vuelta al estado de avance se controla midiendo el tiempo físico real con el reloj del sistema (`time.time()`). Una vez superado el `turn_end_time`, el robot vuelve a marchar hacia delante de manera recta.

### Sistema de desbloqueo (Anti-atasco)
Para resolver situaciones complejas donde el robot choca con esquinas ciegas o irregularidades del mapa, se incluye un contador basado en el desplazamiento euclídeo del robot entre frames:
```python
if prev_x is not None:
    moved = math.hypot(robot_x - prev_x, robot_y - prev_y)
    stuck_timer = 0 if moved > 0.006 else stuck_timer + 1
```
Si el robot pasa más de 50 iteraciones consecutivas sin registrar movimiento real en el espacio, el contador se resetea y el script fuerza una rotación de escape automática con una duración aleatoria de entre 1.5 y 3 segundos.

---

## Construcción del Mapa Probabilístico

### Modelo de Log-Odds
El mapa bidimensional se almacena utilizando una representación basada en log-odds. Esta técnica optimiza el coste computacional al sustituir los productos de matrices probabilísticas por sumas y restas directas, controlando la saturación de las celdas mediante un truncado estricto:

```python
LOG_ODDS_INIT =  0.0
LOG_ODDS_FREE = -0.85  # Valor que se resta a las celdas vacías
LOG_ODDS_OCC  =  0.85  # Valor que se suma a la celda del impacto
LOG_ODDS_MIN  = -3.0   # Límite mínimo para evitar inercia infinita
LOG_ODDS_MAX  =  3.5
```

### Trazado de Rayos con el Algoritmo DDA
Para evitar sobrecargar la CPU del sistema, el mapa de ocupación no se actualiza continuamente. El script evalúa de forma constante la posición del robot y solo dispara la función de mapeado si este ha avanzado una distancia mínima de `0.15` metros desde su última actualización:

```python
do_update = last_upd_x is None or math.hypot(robot_x - last_upd_x, robot_y - last_upd_y) >= MIN_MOVE_DIST
```

Cuando se valida el desplazamiento, se transforma la posición global del robot a coordenadas en píxeles dentro de la matriz (`WebGUI.poseToMap`) y se aplica el algoritmo **DDA** (Digital Differential Analyzer). Este recorre de forma lineal las celdas intermedias atenuando su probabilidad de ocupación, y aplica el incremento positivo únicamente en el píxel final donde impactó el sensor:

```python
# Bucle DDA para vaciar las celdas intermedias del rayo láser
for s in range(steps - 1):
    cx = int(rx_px + s * sx)
    cy = int(ry_px + s * sy)
    if 0 <= cx < MAP_WIDTH and 0 <= cy < MAP_HEIGHT:
        log_odds_map[cy, cx] = np.clip(
            log_odds_map[cy, cx] + LOG_ODDS_FREE,
            LOG_ODDS_MIN, LOG_ODDS_MAX
        )

# Marcamos de forma independiente la celda final como obstáculo ocupado
if 0 <= ex_px < MAP_WIDTH and 0 <= ey_px < MAP_HEIGHT:
    log_odds_map[ey_px, ex_px] = np.clip(
        log_odds_map[ey_px, ex_px] + LOG_ODDS_OCC,
        LOG_ODDS_MIN, LOG_ODDS_MAX
    )
```

### Visualización e Interfaz Gráfica
Para no ralentizar la ejecución del programa con la renderización visual, el mapa numérico se normaliza a una imagen en escala de grises de 8 bits (`uint8`) y se envía al WebGUI de manera controlada **cada 5 iteraciones del bucle principal**:

```python
def log_odds_to_image(lo_map):
    img = np.full((MAP_HEIGHT, MAP_WIDTH), 127, dtype=np.uint8) # Gris: Desconocido
    img[lo_map < -0.5] = 255                                    # Blanco: Libre
    img[lo_map >  0.5] = 0                                      # Negro: Ocupado
    return img
```

---

## Vídeo de Demostración

En el siguiente enlace podéis comprobar en vídeo cómo funciona la navegación autónoma por sectores de tiempo y la velocidad con la que se genera el mapa limpio de la nave industrial:

🔗 [Ver demostración de laser mapping en Drive](https://drive.google.com/file/d/1-0Zua4kqCipsnOHK116XI0c1gxw6ZipC/view?usp=sharing)