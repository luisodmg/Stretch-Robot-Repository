# Stretch Assist Handoff

Fecha: 2026-06-01
Rama: `main`
Ultimo commit remoto revisado: `16288d5 fix: widen stretch assist head search`

## Objetivo

Implementar y probar un flujo de asistencia para Stretch en simulador MuJoCo:

1. Detectar objetos con ArUco.
2. Buscar el objetivo elegido.
3. Aproximarse al objeto.
4. Alinear con camara de muneca.
5. Agarrar, regresar y soltar.

Los objetos configurados son:

- `medicine_box`: ArUco ID `0`
- `glass`: ArUco ID `1`
- `tissue`: ArUco ID `2`

## Archivos principales

- `perception.py`: deteccion ArUco, muestreo de profundidad y proyeccion 3D.
- `state_machine.py`: maquina de estados de Stretch Assist.
- `accessible_ui.py`: selector accesible de objetivo.
- `stretch_assist_config.json`: configuracion runtime.
- `tests/test_perception.py`: pruebas de percepcion.
- `tests/test_state_machine.py`: pruebas de maquina de estados.
- `tests/test_accessible_ui.py`: pruebas de UI accesible.
- `stretch_mujoco/models/scene.xml`: escena MuJoCo con mesa, objetos y marcadores ArUco fisicos modelados.
- `stretch_toolkit/robocasa_config.json`: RoboCasa deshabilitado para usar la escena default.

## Commits relevantes

- `a20a703 feat: add stretch assist implementation`
  - Agrega percepcion, UI, config y maquina de estados inicial.
- `f0812da test: add stretch assist sim markers`
  - Agrega marcadores/objetos en la escena del simulador.
- `95b7556 fix: remove stale stretch assist textures`
  - Quita referencias a texturas PNG inexistentes y usa geometria para los ArUcos.
- `ca92471 fix: improve stretch assist simulator testing`
  - Agrega `--no-teleop`.
  - Mejora manejo de `Ctrl+C`.
- `6f84a1a fix: make stretch assist approach safer`
  - Evita giro de base en busqueda.
  - Hace la aproximacion mas conservadora para no chocar con la mesa.
- `16288d5 fix: widen stretch assist head search`
  - Amplia barrido de cabeza.
  - Agrega `--debug-perception`.

## Validacion realizada

Comandos que han pasado:

```bash
uv run pytest tests
uv run python -m py_compile perception.py state_machine.py accessible_ui.py tests/test_state_machine.py tests/test_perception.py tests/test_accessible_ui.py
```

Resultado mas reciente:

```text
9 passed
```

Tambien se valido que el XML de MuJoCo cargue despues de quitar las texturas faltantes.

## Comando actual para probar

```bash
uv run python state_machine.py --target glass --no-teleop --debug-perception
```

Sin debug:

```bash
uv run python state_machine.py --target glass --no-teleop
```

## Estado observado en simulador

El flujo ya puede:

1. Iniciar MuJoCo.
2. Cargar la escena default con mesa/objetos/ArUcos.
3. Detectar el vaso con la camara de cabeza.
4. Pasar de `SEARCH` a `APPROACH`.
5. Pasar de `APPROACH` a `ALIGN`.

Ejemplo observado:

```text
[Stretch Assist] SEARCH: looking for glass
[Stretch Assist] APPROACH: found Glass
[Stretch Assist] ALIGN: close enough for wrist alignment
```

## Problemas actuales

### 1. El simulador imprime demasiado spam de FPS

Durante toda la ejecucion aparece repetidamente:

```text
WARNING: Passive viewer and camera rendering is below the requested 30.0FPS on the last render.
```

Impacto:

- Ensucia mucho la terminal.
- Hace dificil leer los mensajes reales de Stretch Assist.
- Da la impresion de que el programa esta atorado aunque siga avanzando.

Hipotesis:

- El passive viewer de MuJoCo esta pidiendo 30 FPS, pero la maquina no alcanza esa tasa con viewer + camaras RGB/depth.
- Debe haber una forma de bajar `camera_rate`, desactivar el warning, o correr en modo menos verboso/headless.

Siguiente fix recomendado:

- Buscar en `stretch_mujoco` donde se configura `camera_rate` o donde se imprime el warning.
- Agregar una opcion tipo `--quiet-sim` o configurar el viewer a menor tasa, por ejemplo 10 FPS.
- Alternativamente filtrar/silenciar especificamente ese warning, sin ocultar errores reales.

### 2. Se tarda mucho en completar o queda ciclando en `ALIGN`

Despues de encontrar el vaso y pasar a `ALIGN`, la camara de muneca no ve el ArUco:

```text
[Stretch Assist] perception debug: head_pan=-0.10 head_tilt=-0.77 visible=none
[Stretch Assist] SEARCH: wrist camera lost target
```

Luego vuelve a `SEARCH`, encuentra de nuevo el vaso con la camara de cabeza, vuelve a `ALIGN`, y se repite.

Impacto:

- El robot nunca llega a `GRASP`.
- El sistema parece lento o atorado.

Hipotesis principal:

- Los ArUcos estan arriba de los objetos, visibles para la camara de cabeza, pero no necesariamente visibles para la camara de muneca cuando el brazo intenta alinear.
- La camara de muneca D405 puede estar apuntando a otra direccion o demasiado cerca/lejos.
- La pose de brazo/muneca no se prepara antes de usar la camara de muneca.

Siguiente fix recomendado:

1. Antes de `ALIGN`, mover brazo/lift/wrist a una pose inicial conocida donde la D405 pueda ver la mesa.
2. Agregar debug visual o guardado de frames de camara de muneca.
3. Considerar poner un ArUco adicional visible lateralmente o inclinado, no solo arriba del objeto.
4. Si el objetivo del demo es solo demostrar busqueda y aproximacion, temporalmente saltar `ALIGN/GRASP` o simular grasp despues de detectar con cabeza.

### 3. `Ctrl+C` todavia puede terminar con `ConnectionError`

Aunque se mejoro el manejo de interrupcion, se observo otro traceback si se presiona `Ctrl+C` mientras MuJoCo ya esta cerrandose:

```text
ConnectionError: The Stretch Mujoco Simulator is not running. Use the start() method to start it.
```

Impacto:

- No rompe la logica principal, pero deja mala experiencia al detener la prueba.

Hipotesis:

- El `KeyboardInterrupt` ocurre mientras `step()` esta dentro de una llamada de camara/percepcion, y luego intenta mandar otro comando cuando el simulador ya no esta corriendo.
- `_send_command(..., ignore_connection_error=True)` solo se usa en `finally` y `abort`, pero no en todos los caminos durante shutdown.

Siguiente fix recomendado:

- En `run()`, capturar tambien `ConnectionError` si el simulador se apago durante el loop.
- En `step()`, si `_send_command(command)` falla por `ConnectionError`, transicionar a `ABORTED` sin reventar.
- Mantener errores reales visibles fuera de shutdown si se esta en hardware.

### 4. El debug de percepcion usa el mismo mensaje para cabeza y muneca

En `ALIGN`, el debug imprime:

```text
[Stretch Assist] perception debug: head_pan=... head_tilt=... visible=none
```

Pero en ese estado la deteccion viene de la camara de muneca.

Impacto:

- Confunde al diagnosticar.

Siguiente fix recomendado:

- Pasar un nombre de camara o estado a `_debug_visible_markers`.
- Imprimir `camera=head` o `camera=wrist`.

## Arquitectura actual

Estados actuales:

- `IDLE`
- `SEARCH`
- `APPROACH`
- `ALIGN`
- `GRASP`
- `RETURN`
- `RELEASE`
- `COMPLETE`
- `ABORTED`

Notas:

- La implementacion intenta usar `stretch_toolkit`, no internals directos del simulador.
- `--no-teleop` es necesario para pruebas autonomas, porque el teleop puede cambiar a `MANUAL` y anular comandos.
- `stretch_assist_config.json` se recarga en runtime.

## Recomendacion de siguiente sesion

Prioridad sugerida:

1. Silenciar o reducir el spam de FPS del simulador.
2. Hacer robusto el shutdown con `Ctrl+C`.
3. Diagnosticar la camara de muneca guardando frames o mostrando `camera=wrist`.
4. Agregar una pose de preparacion para `ALIGN`.
5. Decidir si para el demo se debe:
   - completar grasp realista con wrist camera, o
   - simular grasp despues de aproximacion para tener un flujo end-to-end estable.

## Comandos utiles

Actualizar repo:

```bash
git pull origin main
```

Ejecutar tests:

```bash
uv run pytest tests
```

Compilar archivos principales:

```bash
uv run python -m py_compile perception.py state_machine.py accessible_ui.py tests/test_state_machine.py tests/test_perception.py tests/test_accessible_ui.py
```

Probar Stretch Assist:

```bash
uv run python state_machine.py --target glass --no-teleop --debug-perception
```

