# control

Códigos de ejemplo de la asignatura de **control automático** (ETSIAE), escritos en Python con la
librería [python-control](https://python-control.readthedocs.io/).

## Contenido

| Fichero | Qué contiene |
|---|---|
| `simulacion_longitudinal.ipynb` | Cuaderno principal: trimado, linealización, modos propios, actuador, PID de asiento, *altitude hold* y animación de la actitud del avión |
| `dinamica_longitudinal.py` | Modelo no lineal de la dinámica longitudinal de un avión (Cessna 172) y función de trimado |
| `visualizacion.py` | Funciones de gráficas y animación que usa el cuaderno |
| `pid_ejemplos.py` | Ejemplos de control PID: efecto de cada acción |
| `lugar_raices_frecuencia.py` | Lugar de las raíces, Bode, Nyquist y márgenes de estabilidad |
| `pendulo_linealizacion.py` | Trimado y linealización de un péndulo |

`dinamica_longitudinal.py` y `visualizacion.py` deben estar en la misma carpeta que el cuaderno:
si clonas o descargas el repositorio completo, ya lo están.

## Requisitos

- **Python 3.10 o posterior.** Puedes comprobar tu versión con `python --version`
  (en macOS/Linux puede ser `python3 --version`). Si no lo tienes, instálalo desde
  [python.org](https://www.python.org/downloads/) o instala [Anaconda](https://www.anaconda.com/download).
- **Git** (opcional, para clonar y actualizar el repositorio): [git-scm.com](https://git-scm.com/downloads).
  Si no quieres instalarlo, descarga el repositorio como ZIP con el botón verde **Code → Download ZIP**.

## Instalación

### 1. Descargar el repositorio

```bash
git clone https://github.com/igomez666/control.git
cd control
```

### 2. Crear un entorno virtual e instalar las librerías

Un entorno virtual mantiene las librerías de la asignatura separadas del resto de tu ordenador.
Elige **una** de las dos opciones.

**Opción A: `venv` (Python de python.org)**

En Windows (PowerShell o símbolo del sistema):

```bat
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
```

En macOS / Linux:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Opción B: Anaconda / Miniconda** (desde *Anaconda Prompt* en Windows)

```bash
conda create -n control python=3.12
conda activate control
pip install -r requirements.txt
```

Cuando el entorno está activo verás `(.venv)` o `(control)` al principio de la línea de comandos.
**Cada vez que abras una terminal nueva** tendrás que volver a activarlo
(`.venv\Scripts\activate`, `source .venv/bin/activate` o `conda activate control`).

### 3. Comprobar la instalación

```bash
python dinamica_longitudinal.py
```

Debe imprimir la descripción del sistema y el trimado a 60 m/s y 2000 m (α ≈ 0,67°, gases ≈ 0,49).

## Uso

### El cuaderno

Con el entorno activado, desde la carpeta del repositorio:

```bash
jupyter lab
```

Se abre el navegador. Haz doble clic en `simulacion_longitudinal.ipynb` y ejecuta las celdas en orden
(**Run → Run All Cells**, o `Shift + Enter` celda a celda).

Si prefieres **VS Code**: instala las extensiones *Python* y *Jupyter*, abre la carpeta del repositorio,
abre el cuaderno y elige como *kernel* (arriba a la derecha) el entorno `.venv` o `control` que has creado.

### Los scripts

```bash
python pid_ejemplos.py
python lugar_raices_frecuencia.py
python pendulo_linealizacion.py
```

Cada script abre sus gráficas en ventanas; ciérralas para que el script continúe o termine.

### Google Colab (sin instalar nada)

1. En [colab.research.google.com](https://colab.research.google.com) ve a **Archivo → Abrir cuaderno →
   GitHub**, pega `https://github.com/igomez666/control` y elige `simulacion_longitudinal.ipynb`.
2. Inserta una celda **al principio** del cuaderno con este contenido y ejecútala (descarga los ficheros
   `.py` e instala python-control):

   ```python
   !git clone https://github.com/igomez666/control.git
   %cd control
   !pip install -q "control>=0.10.1,<0.11"
   ```

3. Ejecuta el resto de celdas. Los cambios que hagas en Colab no se guardan en el repositorio
   (usa **Archivo → Guardar una copia en Drive** si quieres conservarlos).

### Animaciones (opcional)

La sección 8 del cuaderno anima la actitud del avión. Funciona sin instalar nada más, pero si instalas
**ffmpeg** las animaciones se muestran como vídeo y el cuaderno pesa mucho menos:

- Anaconda: `conda install ffmpeg`
- Windows sin Anaconda: `winget install ffmpeg`
- macOS: `brew install ffmpeg` · Linux: `sudo apt install ffmpeg`

## Actualizar el repositorio

Cuando el profesor suba cambios:

```bash
git pull
```

Si has modificado algún fichero y `git pull` da conflicto, lo más sencillo es trabajar en una **copia**
de los ficheros con otro nombre (por ejemplo `mi_simulacion.ipynb`), así tus cambios nunca chocan con
los del repositorio.

## Problemas frecuentes

- **`ModuleNotFoundError: No module named 'control'`**: el entorno virtual no está activado, o en
  Jupyter/VS Code está elegido otro *kernel*. Activa el entorno y vuelve a lanzar `jupyter lab`, o cambia
  el *kernel*.
- **`ModuleNotFoundError: No module named 'dinamica_longitudinal'`** (o `visualizacion`): el cuaderno no
  está en la misma carpeta que los ficheros `.py`. Ábrelo desde la carpeta del repositorio.
- **`AttributeError: module 'control' has no attribute 'find_operating_point'`** (o `nlsys`): tienes una
  versión antigua de python-control. Actualízala con `pip install -r requirements.txt`.
- **`python` no se reconoce como comando** (Windows): reinstala Python marcando la casilla
  *Add python.exe to PATH*, o usa `py` en lugar de `python`.
