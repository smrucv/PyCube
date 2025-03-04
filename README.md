# Rubik's Cube Solver & Simulator

## Descripción

Este proyecto es un simulador 3D interactivo del Cubo de Rubik desarrollado en Python con `pygame` y `PyOpenGL`. Además, incorpora la librería `kociemba` para resolver automáticamente el cubo utilizando el algoritmo de Kociemba.

## Características

- Simulación visual 3D del Cubo de Rubik.
- Permite rotar libremente la cámara con el teclado.
- Movimientos interactivos del cubo usando las teclas.
- Función de mezclado aleatorio (`scramble`).
- Resolución automática mediante el algoritmo de Kociemba (En Proceso).

## Requisitos

Asegúrate de tener instaladas las siguientes dependencias:

```bash
pip install pygame PyOpenGL numpy kociemba
```

## Controles

- **Rotación del Cubo:**
  - `W` / `w`: Rotar la cara Blanca (White)
  - `Y` / `y`: Rotar la cara Amarilla (Yellow)
  - `G` / `g`: Rotar la cara Verde (Green)
  - `B` / `b`: Rotar la cara Azul (Blue)
  - `R` / `r`: Rotar la cara Roja (Red)
  - `O` / `o`: Rotar la cara Anaranjada (Orange)
- **Cámara:**
  - `Flechas izquierda/derecha`: Girar la cámara horizontalmente.
  - `Flechas arriba/abajo`: Girar la cámara verticalmente.
  - `+` / `-`: Acercar o alejar la cámara.
- **Funciones adicionales:**
  - `S`: Mezclar el cubo aleatoriamente.
  - `0`: Reiniciar el cubo a su estado resuelto.
  - `V`: Resolver el cubo automáticamente usando el solver de Kociemba.

## Uso

Para ejecutar el simulador, ejecuta el siguiente comando:

```bash
python rubiks_cube.py
```

Interacciona con el cubo mediante los controles mencionados. Para resolverlo automáticamente, presiona `V`.

## Notas

- El solver se basa en la conversión del estado del cubo a una cadena en la notación facelet esperada por `kociemba`.
- La animación y la lógica del cubo se manejan mediante transformaciones matriciales en 3D.
- Si experimentas errores con `kociemba`, asegúrate de que el estado facelet generado sea válido y tenga 54 caracteres.

## Capturas de pantalla

*(Opcional: Agregar capturas de pantalla del simulador en funcionamiento.)*

## Licencia

Este proyecto se distribuye bajo la licencia MIT.

