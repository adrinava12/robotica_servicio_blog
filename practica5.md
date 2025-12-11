# Mapeado por Laser

## Resumen

En esta practica, el objetivo es crear un mapa desde cero usando el laser del
que dispone el robot. En cuanto a la navegacion se deberia de usar un algoritmo de
cobertura como el de la [practica 1](practica1.md).

## Escaneo Laser

El enfoque principal de esta practica es el escaneo laser del entorno y crear un mapa
con dichas observaciones. Ademas, para aumentar el realismo, las funciones de las
que disponemos para obtener la posicion del robot, tiene ruido.

1. Obtener la posicion del robot, y pasarlo a coordenadas del mapa:

```python
    odom = HAL.getOdom()
    robot_x = odom.x
    robot_y = odom.y
    robot_yaw = odom.yaw

    obot_map_coords = WebGUI.poseToMap(robot_x, robot_y, robot_yaw)
    robot_map_x = int(robot_map_coords[0])
    robot_map_y = int(robot_map_coords[1])
```

2. Comenzar a procesar el laser:

Debemos de procesar tambien los valores que superen el valor maximo que registra el laser,
lo mismo para los minimos.

```python
    for i in range(0, 360): 
        dist = laser.values[i]
        
        # Ignorar valores fuera de rango
        if dist >= laser.maxRange:
            dist = laser.maxRange

        if dist <= laser.minRange:
            dist = laser.minRange
```

3. Calcular el punto final del rayo

Ahora para cada medida debemos de pasar el final del rayo del laser a coordenadas 
del mapa.

```python
    # Calcular punto final del rayo
    angle = robot_yaw + math.radians(i)
    world_x = robot_x + dist * math.cos(angle)
    world_y = robot_y + dist * math.sin(angle)

    # Convertir a coordenadas del mapa
    point_map = WebGUI.poseToMap(world_x, world_y, 0)
    point_x = int(point_map[0])
    point_y = int(point_map[1])
```

4. Marcar como obstaculo en el mapa los finales de rayo que esten dentro del rango de funcionamiento del laser:

```python
    # Marcar como obstaculo puntos que no son maximos del laser
    if dist > laser.minRange and dist < laser.maxRange:
        # Marcar obstáculo
        if 0 <= point_x < 1500 and 0 <= point_y < 970:
            mapa[point_y, point_x] = OBSTACLE
```

5. Marcar los pixeles desde el robot hasta el final del laser como libres:

    1. Calcular cuanto sera el salto entre cada paso:
    ```python
        dx = x2 - x1
        dy = y2 - y1
        steps = max(abs(dx), abs(dy))
        
        if steps == 0:
            return
        
        x_increment = dx / steps
        y_increment = dy / steps
    ```

    2. Marcar libre los pixeles en funcion de los indices calculados
    ```python
        x = float(x1)
        y = float(y1)
        
        for i in range(int(steps)):
            ix = int(x)
            iy = int(y)
            
            if 0 <= ix < 1500 and 0 <= iy < 970:
                # Solo marcar como libre si no es obstáculo
                if mapa[iy, ix] != OBSTACLE:
                    mapa[iy, ix] = FREE
            
            x += x_increment
            y += y_increment
    ```

## Navegacion

La navegacion de esta practica se basa en la navegacion usadada en la [practica 1](practica1.md)
que usaba un algoritmo de cobertura para cubrir todo el espacio del mapa, 
la unica diferencia es que en este caso se debe de tener en cuenta que el mapa
no esta completo y que por lo tanto pueden surgir nuevos obstaculos que al crear 
las rutas no estaban.

