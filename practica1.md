# Práctica 1: Programacion de una aspiradora localizada

## Introduccion
En esta practica se pide programar una aspiradora capaz de autolocalizarse, 
para ello se han seguido los siguientes pasos:

 - Registro del mapa
 - Creación de la rejilla de navegación
 - Planificación de ruta siguiendo algoritmo de cobertura BSA
 - Pilotaje reactivo para ejecutar la ruta planificada.

## Paso 1 — Registro del mapa (calibración / transformación afín)

### Objetivo
El objetivo de este paso es **relacionar las coordenadas de la imagen (en píxeles)** con las **coordenadas reales del mundo de Gazebo (en metros)**.  
Esto permite que el robot sepa en qué parte del mapa se encuentra y pueda convertir sus movimientos del mundo físico al mapa y viceversa.

### Concepto
Se usa una **transformación afín 2D**.  

```python
M = [ [a, b, tx],
      [c, d, ty],
      [0, 0, 1] ]
```
donde:

- `a` y `d` representan la escala y orientación en los ejes X e Y:
    - `a` → escala + posible rotación en el eje X
    - `d` → escala + posible rotación en el eje Y

- `b` y `c` representan la inclinación o mezcla de ejes:
    - `b` → cuánto el eje Y afecta al eje X
    - `c` → cuánto el eje X afecta al eje Y

- `tx` y `ty` son los desplazamientos en píxeles para mover la imagen resultante:
    - `tx` → desplazamiento horizontal

    - `ty` → desplazamiento vertical

Para obtener estos parametros en la funcionm `compute_affine_matrix` se plantea un sistema de ecuacion con estos parametros como incognitas, y los puntos de referencia obtenidos del gazebo y su correspondencia en pixeles, y se resuelve el sistema por el metodo de minimos cuadrados.
### Funciones principales
- `compute_affine_matrix(ref_pixels, ref_gazebo)`
- `gazebo_to_image(x, y, M)`
- `image_to_gazebo(u, v, M)`

### Detalle de implementación
1. Se definen listas de puntos de referencia, que tomamos directamente de puntos identificables facilmente en la imagen, se necesitan al menos 3 puntos aunque es preferible mas, ademas deben de estar bien distribuidos a lo largo del escenario:
   ```python
   ref_pixels = [[u1, v1], [u2, v2], ...]
   ref_gazebo = [[x1, y1], [x2, y2], ...]
   ```

2. Se construye un sistema lineal para estimar los parámetros `a, b, c, d, tx, ty`.

3. Se resuelve por mínimos cuadrados con `np.linalg.lstsq(A, B)`.

4. Se forma la matriz M de 3×3.

5. Las funciones `gazebo_to_image` y `image_to_gazebo` aplican la transformación y su inversa, para ello se multiplica la matriz M por el vector de puntos de gazebo en caso de querer pasar a la imagen, y se usa la inversa de M en caso de querer pasar de la imagen al gazebo

## Paso 2 - Creación de la rejilla (discretización del mapa)

### Objetivo

Convertir el mapa (imagen) en una rejilla de celdas discretas, donde cada celda representa una zona del entorno: libre, sucia u ocupada por un obstáculo.
Esto facilita aplicar algoritmos de búsqueda y planificación de trayectorias.

### Proceso de preprocesamiento
1. Cargar el mapa:
```python
map_png = WebGUI.getMap('/resources/.../mapgrannyannie.png')
```

2. Convertir a escala de grises → cv2.cvtColor

3. Binarizar con cv2.threshold:
    - 0 → libre
    - 255 → obstáculo

4. Erosionar con cv2.erode para engrosar obstáculos (seguridad de colisión).

5. Dividir el mapa en celdas y asignar valores.

### Función `create_grid(eroded_map)`

1. Se crea un array con el numero de pixeles de ancho de la imagen entre el numero de pixeles de ancho de las celdas(decidido por nosotros), eso define el numero de celdas a lo ancho, se hace lo mismo en vertical.

2. Se recorre el mapa y se marca cada bloque:
    ```python
    if np.any(block == 0):
        grid_map[i, j] = OCCUPY_GRID
    else:
        grid_map[i, j] = DIRTY_GRID
    ```

Con esto cada celdilla queda marcada como ocupada (cualquiera de sus pixles esta ocupado), o como sucia (celda libre disponible para limpiar)
