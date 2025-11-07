# Práctica 3: Autoaparcamiento

## Introducción

En esta práctica se pide programar un coche capaz de aparcar de manera autónoma.  
Para su implementación, se ha diseñado una **máquina de estados jerárquica**, en la cual un estado principal contiene a su vez otras dos máquinas de estados.  
La estructura general es la siguiente:

* **Search parking**
  * Approach cars  
  * Align with street  
  * Search space  
* **Parking**
  * Initial park  
  * Mid park  
  * Final park  
  * End  

---

## Estado 1: Search parking

En este estado se realizan todas las operaciones necesarias para acercarse a la línea de coches, alinearse con la calle y buscar un hueco libre donde aparcar.  
Para lograrlo, este estado se divide en tres subestados, cada uno encargado de una de las tareas mencionadas.

---

### Estado 1.1: Approach cars

El objetivo de este subestado es **acercarse lo máximo posible a la línea de coches aparcados**.  
Esto se hace para que, una vez alineado con la calle, el vehículo solo deba avanzar hacia adelante para buscar hueco.

Para acercarse, se toma la **medida mínima del láser derecho**, que indica la distancia más corta a los objetos detectados (coches o paredes):

```python
d_min = float('inf')
for i in range(180):
    d = right_laser.values[i]
    if right_laser.minRange < d < right_laser.maxRange:
        d_min = min(d_min, d)
```

Una vez obtenida la medida mínima, se utiliza un **controlador proporcional (P)** para reducir esa distancia hasta alcanzar un umbral que permita iniciar la alineación.  

En caso de que el coche no detecte ningún obstáculo cercano (por ejemplo, si se encuentra en una calle sin coches ni paredes), avanza hacia adelante a menor velocidad:

```python
if d_min == float('inf'):
    print("No se detecta pared o coches a la derecha.")
    HAL.setV(VELOCIDAD_LINEAL / 2)
    HAL.setW(0)
```

---

### Estado 1.2: Align with street

Este estado es probablemente el más importante, ya que el éxito del resto del proceso depende de que el vehículo quede correctamente alineado con la calle.  

Para ello, se utiliza el **método SVD (Singular Value Decomposition)** de la librería *NumPy*, que permite determinar la dirección dominante de los puntos detectados por el láser, y así conocer el ángulo de la calle.  
Posteriormente, mediante un **controlador PD**, se ajusta la orientación del coche hasta alinearla con dicha dirección.

```python
measures = []
for index in range(180):
    measure = right_laser.values[index]
    if right_laser.minRange < measure < right_laser.maxRange:
        measures.append(lidar_to_car(index, measure, "right"))

if len(measures) < 5:
    # Demasiados puntos inválidos, avanza lento y sin girar
    HAL.setV(VELOCIDAD_LINEAL)
    HAL.setW(0)
    return ALIGN_WITH_STREET

# --- Calcular la dirección dominante con SVD ---
A = np.array(measures)
centroid = A.mean(axis=0)
A_centered = A - centroid
U, S, Vt = np.linalg.svd(A_centered)
direction = Vt[0]

# Ángulo de los coches respecto al eje X del coche
angle = np.arctan2(direction[1], direction[0])
error = np.degrees(angle)  # Queremos que sea 0°
```

Se pasa al siguiente estado cuando el error de orientación sea **inferior a 1 grado**, lo que garantiza que el vehículo esté correctamente alineado.

---

### Estado 1.3: Search space

El objetivo de este estado es **avanzar en paralelo con la calle**, buscando un hueco donde el vehículo pueda aparcar.  

Para desplazarse en paralelo se aprovecha la orientación obtenida en el estado anterior.  

La detección del hueco se realiza mediante un **rectángulo definido en coordenadas relativas al coche**, con dimensiones ligeramente superiores al tamaño del vehículo.  
El objetivo es comprobar que **ninguna medida de los tres láseres (frontal, trasero y lateral)** se encuentra dentro de dicho rectángulo.  
Si está completamente libre, significa que el hueco es apto para aparcar.

El rectángulo se coloca en paralelo al coche y desplazado hacia atrás la longitud del vehículo, de manera que, cuando se detecta el hueco, el coche ya se encuentra correctamente posicionado para iniciar las maniobras.

Primero se convierten todas las medidas de los láseres a coordenadas relativas al coche:

```python
for index in range(len(right_laser.values)):
    dist_r = right_laser.values[index]
    dist_b = back_laser.values[index]
    dist_f = front_laser.values[index]

    # Comprobamos si son valores válidos antes de convertirlos y guardarlos
    if right_laser.minRange < dist_r < right_laser.maxRange:
        coors_right_laser.append(lidar_to_car(index, dist_r, lidar='right', sensor_offset=(0.0, 0.5)))

    if back_laser.minRange < dist_b < back_laser.maxRange:
        coors_back_laser.append(lidar_to_car(index, dist_b, lidar='rear', sensor_offset=(0.5, 0.0)))

    if front_laser.minRange < dist_f < front_laser.maxRange:
        coors_front_laser.append(lidar_to_car(index, dist_f, lidar='front', sensor_offset=(-0.5, 0.0)))
```

Después, basta con una simple comprobación para verificar si algún punto cae dentro del rectángulo:

```python
if esquina1[0] > x > esquina2[0] and esquina1[1] > y > esquina2[1]:
```

---

## Estado 2: Parking

Una vez encontrado un hueco libre, comienza la fase de **maniobra de aparcamiento**, dividida en cuatro etapas:

* **Initial park**  
* **Mid park**  
* **Final park**  
* **End**

---

### Estado 2.1: Initial park

En esta etapa se realiza la **primera maniobra de aparcamiento**.  
El vehículo gira completamente hacia la derecha y da marcha atrás, controlando el **ángulo del coche y la distancia mínima medida por el láser trasero**.  
La maniobra finaliza cuando el ángulo sea inferior a 80° o no haya medidas válidas.

```python
# Condición para continuar con esta maniobra
if angulo > 80 or d_min == float('inf'):
```

---

### Estado 2.2: Mid park

En esta etapa el vehículo **da marcha atrás en línea recta** para terminar de entrar en el hueco y evitar quedar invadiendo la calzada.  
La velocidad angular es nula, ya que únicamente se desea retroceder.

---

### Estado 2.3: Final park

En este estado se completa la maniobra de aparcamiento.  
El coche gira hacia la izquierda mientras continúa dando marcha atrás, hasta que el ángulo medido por el láser frontal sea superior a 90°.

```python
# Condición para continuar con esta maniobra
if angulo < 90 or d_min == float('inf'):
```

---

### Estado 2.4: End

Este es el **estado final** del proceso de aparcamiento.  
El coche avanza ligeramente hacia adelante hasta mantener una distancia de aproximadamente 1.5 metros con el vehículo delantero, corrigiendo el ángulo con un pequeño giro a la derecha para quedar perfectamente alineado.

---

## Vídeo de demostración

[Ver demostración de autoaparcamiento](https://drive.google.com/file/d/1Y6qK7zY5-AiOix8wFWdBmWhgKd12eXRh/view?usp=sharing)