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