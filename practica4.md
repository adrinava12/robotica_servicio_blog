# Practica 4: Robot de Logistica

## Introduccion

El objetivo de esta práctica es programar dos robots de logística que trabajan en un almacén moviendo estanterías. Para ello, utilizamos la librería OMPL para planificar la ruta de los robots.

Debemos lograr que los robots transporten estanterías de un punto a otro del almacén de forma eficiente y segura, evitando colisiones con obstáculos y otras estanterías.

Los robots con los que trabajaremos son:

- *Robot holonómico*: dimensiones 0.40 m × 0.40 m (vacio) y 2.0 m × 0.75 m (cargado).
- *Robot Ackermann*: dimensiones 0.72 m × 0.32 m (vacio) y 2.0 m × 0.75 m (cargado).

## Aspectos generales

En esta sección se describen los aspectos comunes de la práctica, sin importar el tipo de robot.

### Tratamiento del mapa

Primero, se carga el mapa del almacén y se binariza usando la librería *OpenCV*.
Esto facilita la identificación de obstáculos frente a zonas libres.

Esto nos permitirá comprobar de manera eficiente si un estado (posición del robot) es válido dentro del planificador.

### Configuracion del planficador
Para la planificación de trayectorias se utiliza la librería **OMPL (Open Motion Planning Library)**. OMPL proporciona diversas estructuras de datos y algoritmos para generar caminos seguros en espacios complejos, evitando colisiones.  

En el caso del roboto holonomico, el espacio de estados seleccionado es **SE2**, que representa:

- La posición `(x, y)` del robot en el plano.
- La orientación `θ` del robot respecto a un eje de referencia. 

En el caso del robot Ackermann  se hace uso del **Reeds-Shepp State Space**, que representa:

- La posición `(x, y)` del robot en el plano.  
- La orientación `θ` del robot. 

El **Reeds-Shepp** es una extensión de la geometría de **Dubins**, que permite al robot moverse tanto hacia adelante como hacia atrás, generando trayectorias curvas continuas con un radio mínimo de giro. Esto es fundamental para robots tipo automóvil que no pueden girar en el lugar.  

Para la planificación concreta se utiliza el algoritmo **RRT\***, un planificador basado en muestreo que explora el espacio de estados de forma aleatoria y gradual, extendiendo nodos hasta conectar el punto inicial con el objetivo. Este algoritmo se elige por su capacidad de:

- Manejar espacios de estados de varias dimensiones.
- Encontrar caminos iniciales rápidos.
- Optimizar gradualmente la trayectoria para reducir la longitud del camino.  

El planificador requiere varios parámetros y elementos clave:

- **Start y goal**: el punto de inicio y el destino de la ruta.  
- **Límites del mapa**: que definen el área de movimiento del robot.  
- **Función de validez de estados (`is_state_valid`)**: que verifica si la posición del robot está libre de colisiones.  
- **Rango de expansión (`setRange`)**: que determina la longitud de los pasos que da el planificador entre nodos, afectando la suavidad y resolución del camino.  
- **Bias hacia el objetivo (`setGoalBias`)**: que aumenta la probabilidad de muestrear el punto objetivo, acelerando la convergencia del planificador.  

### Funcion is_state_valid

Una parte fundamental de la planificación de trayectorias es asegurarse de que el robot no colisione con obstáculos ni salga de los límites del mapa. Para esto se utiliza la función is_state_valid, que OMPL llama cada vez que evalúa un estado potencial durante la planificación.

1. Obtener coordenadas del estado:

`x` y `y` representan la posición del robot en el mapa real (coordenadas continuas).

```python
x = state.getX()
y = state.getY()
```

2. Comprobar límites del mapa:

Se descartan estados que estén fuera de los límites definidos (`Xmin`, `Xmax`, `Ymin`, `Ymax`).
```python
if x < Xmin or x > Xmax or y < Ymin or y > Ymax:
    return False
```

3. Definir el “footprint” del robot:

Se calcula un rectángulo que representa el robot usando su longitud `ROBOT_L` y ancho `ROBOT_W`.
```python
L = ROBOT_L / 2
W_ = ROBOT_W / 2

# vértices del rectángulo del robot
footprint = [
    (x - L, y - W_),
    (x + L, y - W_),
    (x + L, y + W_),
    (x - L, y + W_)
]
```

4. Revisar vértices del robot:

Cada vértice se transforma a coordenadas del mapa (`world_to_map`) y se comprueba si toca un obstáculo (`binary_map[my, mx] == 0`).
Si alguno está fuera de los límites o sobre un obstáculo, el estado se considera inválido.

```python
# revisa si los vértices colisionan con obstáculos
for px, py in footprint:
    mx, my = world_to_map(px, py)
    if mx < 0 or mx >= W or my < 0 or my >= H:
        return False
    if binary_map[my, mx] == 0:
        return False

```

5. Revisar puntos interiores:

Para mayor seguridad, se muestrean varios puntos dentro del rectángulo del robot y se comprueba si colisionan. Esto evita que el robot “pase por encima” de obstáculos con esquinas o bordes.

```python
# revisa puntos interiores del robot
for sx in np.linspace(x - L, x + L, 5):
    for sy in np.linspace(y - W_, y + W_, 5):
        mx, my = world_to_map(sx, sy)
        if mx < 0 or mx >= W or my < 0 or my >= H:
            return False
        if binary_map[my, mx] == 0:
            return False
```

6. Retorna validez:
- `True`: estado libre y seguro
- `False`: colision o fuera de los limites

Esta funcion es importante porque:

- Permite que el planificador omita estados imposibles, evitando caminos que terminen en colisión.
- Adapta la planificación a robots de distinto tamaño, simplemente cambiando `ROBOT_L` y `ROBOT_W`.
- Se puede usar tanto para robots holonómicos como para Ackermann, asegurando que el “footprint” sea respetado en ambos casos.

### Visualizacion de estanterias

Durante la práctica necesitamos representar gráficamente las estanterías en el mapa a medida que los robots las mueven. Para ello usamos dos funciones principales:

- `draw_rectangle_corners`: esta función se encarga de dibujar las esquinas de la estantería en el nuevo lugar donde se ha colocado. En lugar de dibujar un rectángulo completo que podría ocultar detalles del mapa, solo se dibujan las esquinas en forma de "L", permitiendo visualizar la posición de la estantería de forma clara y manteniendo visibles los obstáculos y el camino del robot.

- `lean_map`: cuando el robot recoge la estantería de su posición inicial, esta función borra el rectángulo correspondiente a esa ubicación. Así, el mapa refleja correctamente que la estantería ha sido retirada, evitando confusiones para el planificador y para la visualización de la simulación.

Estas dos funciones trabajan de forma complementaria para mantener el mapa actualizado: `clean_map` elimina lo que ya no está, y `draw_rectangle_corners` muestra la nueva posición. Esto facilita seguir el progreso de los robots y comprobar visualmente que las estanterías se han movido correctamente.

## Robot Holonomico 

### Control del robot holonómico

Una vez generada la trayectoria, el robot necesita seguirla con precisión. Para esto se utiliza un **controlador P**.  

El controlador funciona de la siguiente manera:

1. **Lectura de la posición actual**: se obtiene la posición `(x, y)` y la orientación `θ` del robot a partir del módulo HAL, que proporciona los datos de estado del robot.  
2. **Cálculo de errores**:
   - **Error lineal**: distancia entre la posición actual del robot y el waypoint objetivo.  
   - **Error angular**: diferencia entre la orientación deseada (calculada con `atan2(dy, dx)`) y la orientación actual del robot. Se normaliza a `[-π, π]` para evitar saltos de ángulo.  
3. **Cálculo de velocidades**:
   - **Velocidad lineal (V)**: proporcional al error lineal. Para evitar movimientos bruscos, se limita con un máximo y mínimo.  
   - **Velocidad angular (W)**: proporcional al error angular. También se limita a un rango seguro.  

4. **Actuación del robot**: las velocidades calculadas se envían al robot mediante `HAL.setV(V)` y `HAL.setW(W)`. El robot se mueve directamente hacia el siguiente waypoint.  

5. **Detención en el waypoint**: cuando el error lineal es menor que un umbral (`0.15 m`), se considera que el waypoint ha sido alcanzado y se detiene el robot. Esto garantiza que el robot no oscile alrededor del punto objetivo y pueda pasar al siguiente waypoint sin problemas.  

## Robot Ackermann


A diferencia del robot holonómico, el robot Ackermann tiene restricciones de movimiento debido a su geometría tipo automóvil. Este robot no puede moverse lateralmente y solo puede desplazarse hacia adelante o hacia atrás siguiendo una curva definida por su ángulo de dirección. Esto implica que la planificación y el control requieren estrategias más sofisticadas para asegurar que la trayectoria sea realizable físicamente.  

### Control del robot Ackermann

El control de un robot Ackermann es más complejo que el de un robot holonómico debido a sus restricciones cinemáticas. Para el seguimiento de trayectoria se utiliza un **controlador tipo PID adaptado a Ackermann**, con las siguientes características:

1. **Lectura de estado**: se obtiene la posición `(x, y)` y la orientación `θ` del robot a través del módulo HAL.  
2. **Determinación de la dirección de marcha**:  
   - Se calcula el ángulo hacia el waypoint objetivo.  
   - Si el ángulo es mayor a π o menor a -π, se activa la **marcha atrás** para que el robot pueda seguir la trayectoria sin realizar giros imposibles.  
   - Se incluyen márgenes de tolerancia para evitar que el robot oscile entre avanzar y retroceder en bucle cerca del límite.  

3. **Cálculo de errores**:
   - **Error lineal**: distancia al waypoint.  
   - **Error angular**: diferencia entre la orientación deseada y la actual, normalizada a `[-π, π]`.  

4. **Control proporcional-derivativo (PD)**:
   - **Velocidad lineal (V)**: proporcional al error lineal y ajustada según si el robot debe ir hacia adelante o atrás. Se limita para no exceder la velocidad máxima segura.  
   - **Velocidad angular (W)**: proporcional al error angular y su derivada, para suavizar giros y garantizar que el robot pueda seguir las curvas planificadas.  

5. **Seguimiento de waypoints**:
   - La trayectoria se subdivide en waypoints separados, y el robot sigue cada punto secuencialmente.  
   - Se utiliza un umbral de distancia para considerar un waypoint alcanzado (`~0.15 m`). Esto evita que el robot oscile y quede atrapado intentando ajustar su posición infinitamente.  

6. **Ajuste final de orientación**:
   - Una vez alcanzada la posición objetivo, el robot ajusta su orientación final para alinearse correctamente con el objetivo.  
   - Este paso es esencial para robots que deben colocar o recoger estanterías, ya que la orientación determina cómo se alineará el robot con la estantería.  

## Videos demostracion

[Video Holonomico](https://drive.google.com/file/d/1EiAAF53UhwZm30pZ88o7OE21P5l0CQ0_/view?usp=sharing)

[Video Ackermann](https://drive.google.com/file/d/14kHqcmf-9x4_pkzyUA3UQ5pHuJXxMHA3/view?usp=sharing)