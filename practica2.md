# Práctica 2: Rescate de personas

## Introduccion

En esta practica se pide programar un dron para la localizacion de supervivientes
en alta mar. Para la realizacion de esta practica, he creado una maquina de estados
con los siguientes estados:

- Aproximacion a la zona de busqueda
- Busqueda
- Vuelta a la lancha

Para detectar a los supervivientes durante el estado de busqueda, se ha usado
el detector de caras de OpenCV HaarCascade.

## Estado 1: Aproximacion a la zona de busqueda

Este estado es muy simple, consiste en despegar de la lancha, e ir a la zona 
que se nos ha marcado de busqueda.

Cambiaremos de estado una vez hallamos llegado a la zona de busqueda, para saberlo
lo comprobamos de la siguiente manera:

```python
    pose = HAL.get_position()
    x, y, z = pose
    x_obj, y_obj, z_obj = SURVIVORS_EST_POS
    distancia = math.sqrt((x_obj-x)**2 + (y_obj-y)**2)

    # Ya se ha llegado, pasar a estado de busqueda
    if distancia < 0.25:
        return SEARCH

    # Todavia no se ha llegado, seguir en el mismo estado
    else:
        return GO2SEARCH_ZONE
```

## Estado 2: Busqueda de supervivientes


## Estado 3: Vuelta a la lancha

Este estado se alcanzara una vez la bateria del dron baje de un nivel critico,
una vez el nivel de bateria sea muy bajo, el dron dejara la busqueda, y comenzara la vuelta a la lancha, despues de llegar, el dron aterrizara en la lancha.

## Video

[Ver demostracion busqueda supervivientes](https://drive.google.com/file/d/1wdCD7wgKnwyFG_e4t5bLTjfIXgq9u-cP/view?usp=sharing)