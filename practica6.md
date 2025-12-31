# Lolizacion por marcadores visuales

## Resumen

En esta practica se pide lozalizar nuestro robot usando marcadores AprilTags,
cuando estos no son visibles usaremos la odometria para saber aproxidamente donde
estamos.

## Navegacion

En esta practica no se pedia ningun requisito para la navegacion por lo tanto
he hecho una navegacion aleatoria con la intencion de que a veces vea los marcadores
y otras veces no para poder probar bien los cambios entre localizarse con marcadores
o con odometria.

## Odometria

Para la odometria lo que he hecho es guardarme en una variable la posicion previa
que hemos estimado, ya sea con odometria o con los marcadores.
Cogemos la posicion actual que nos da la odometria y se la restemos a la poscion previa
de tal manera que conoceremos el incremento que ha habido en posicion, de tal manera que 
tendremos nuestra nueva posicion estiamada.

```python
def update_with_odom():
    global prev_pose

    image = HAL.getImage()
    WebGUI.showImage(image)

    x, y, yaw = HAL.getOdom().x, HAL.getOdom().y, HAL.getOdom().yaw

    dx = x - prev_pose[0]
    dy = y - prev_pose[1]
    dyaw = yaw - prev_pose[2]

    prev_pose = [prev_pose[0] + dx, prev_pose[1] + dy, prev_pose[2] + dyaw]
```

## Localizacion por marcadores visuales

El primer paso para poder localizarnos con marcadores visuales es detectarlos, 
para eso usamos el detector que nos da la libreria pyapriltags.

```python
detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")
```

Despues lo que hacemos es recorrer todas las detecciones que nos ha dado, y quedarnos
con la que mayor area tiene, es decir con la mas cercana.

```python
    linea1 = math.sqrt((ptA[0] - ptB[0])**2 + (ptA[1] - ptB[1])**2)
    linea2 = math.sqrt((ptC[0] - ptD[0])**2 + (ptC[1] - ptD[1])**2)
    Area = linea1 * linea2
    if max_area[0] < Area:
        max_area = (Area, r.tag_id)
        puntoA = ptA
        puntoB = ptB
        puntoC = ptC
        puntoD = ptD
```

Para poder ver mejor las deteccion, las marcamos con un rectangulo verde en la imgagen

Una vez tenemos la deteccion mas cercana, debemos de pasar a usar PNP, en este caso
he usado el metodo SOLVEPNP_ITERATIVE.
Para usar PNP debemos de saber el tamaño de los tags, asi como los pixeles de la imagen
donde estan las esquinas, ademas tambien tenemos que saber los valores intrinsecos de la camara.

```python
    # Puntos 3D del tag 
    object_points = np.array([
        [-TAG_SIZE/2, -TAG_SIZE/2, 0],
        [ TAG_SIZE/2, -TAG_SIZE/2, 0],
        [ TAG_SIZE/2,  TAG_SIZE/2, 0],
        [-TAG_SIZE/2,  TAG_SIZE/2, 0]
    ], dtype=np.float32)

    # Puntos 2D detectados en la imagen
    image_points = np.array(detected_tag, dtype=np.float32)

    # Matriz cámara aproximada
    h, w = image.shape[:2]
    focal = w  # focal aproximada
    center = (w/2, h/2)
    camera_matrix = np.array([
        [focal, 0, center[0]],
        [0, focal, center[1]],
        [0, 0, 1]
    ], dtype=np.float32)
    dist_coeffs = np.zeros(4)

    # Resolver PnP
    success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix, 
            dist_coeffs,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
```

Esto nos dara dos matrices, una de rotacion y otra de traslacion, con las que podremos
pasar de la posicion del robot a la del tag respecto del robot, pero nosostros no queremos
eso, sino que queremos la posicion del robot respecto del mundo, es por eso que
ahora debemos de hacer una seria de transformaciones.

Lo primero que hago es invertir la transformacion para asi tener del Tag->Camara.

```python
 # Invertir la transformación (Cámara respecto al Tag)
R_tc = R_ct.T
t_tc = -R_tc @ tvec
```
Despues debemos de alinear los Ejes del sistema de referencia del Tag con el del mundo

```python
   R_swap = np.array([
        [1, 0, 0],  # X_tag → X_world
        [0, 0, 1],  # Y_tag → Z_world
        [0, 1, 0]   # Z_tag → Y_world
    ])
    R_tc_corrected = R_swap @ R_tc
    t_tc_corrected = R_swap @ t_tc
```
Una vez hecho esto podemos crear la transformacion del Mundo al Tag, esta se construye
a partir de la poscion que se nos da de la tag.

```python
    yaw_tag = tag_map_pose[3]  # yaw del tag en rad
    R_wt = np.array([
        [ np.cos(yaw_tag), -np.sin(yaw_tag), 0],
        [ np.sin(yaw_tag),  np.cos(yaw_tag), 0],
        [ 0, 0, 1]
    ])
```

Y por ultimo hacemos la multiplicacion de matrices para pasar del mundo al robot
y asi obtener su posicion.

```python
    R_wc = R_wt @ R_tc_corrected
    t_wc = R_wt @ t_tc_corrected + t_wt
```

Con esto obtenemos la traslacion y rotacion para pasar del mundo al robot, 
obteniendo asi su posicon extrayendola de las matrices.

```python
x_r = t_wc[0, 0]
y_r = t_wc[1, 0]
yaw_r = np.arctan2(R_wc[1, 0], R_wc[0, 0])
```
## Vídeo de demostración

[Ver demostración de localizacion por marcadores visuales](https://drive.google.com/file/d/1EKaYxGU_6oLP-mKEGnM-biG9AAGCKYPE/view?usp=sharing)