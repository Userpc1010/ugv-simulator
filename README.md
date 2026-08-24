# UGV Simulator

Базовый симулятор беспилотного наземного транспортного средства (UGV) с физикой, навигацией и несколькими контроллерами движения.

![Screenshot 1](Screenshot%20from%202026-08-24%2014-20-30.png)

![Screenshot 2](Screenshot%20from%202026-08-24%2014-20-52.png)

![Screenshot 3](Screenshot%20from%202026-08-24%2014-21-02.png)

![Screenshot 4](Screenshot%20from%202026-08-24%2014-21-24.png)

![Screenshot 5](Screenshot%20from%202026-08-24%2014-24-54.png)

![Screenshot 6](Screenshot%20from%202026-08-24%2014-25-25.png)

![Screenshot 7](Screenshot%20from%202026-08-24%2014-25-30.png)

## Возможности

- 🚗 **Физика автомобиля**
  - Подвеска с пружинами и демпферами
  - Модель шин Pacejka
  - Аккермановское рулевое управление
  - Дифференциалы

- 🗺️ **Террайн и поверхности**
  - Карта высот 4097×4097
  - 5 типов поверхностей: земля, лес, горная дорога, проходы, болото
  - Различное сцепление и сопротивление качению

- 🧭 **Навигация**
  - Costmap на основе уклонов
  - State Lattice планировщик
  - Локальное окно 800×800

- 🎮 **Контроллеры**
  - **Manual** — ручное управление
  - **MPPI** — Model Predictive Path Integral
  - **RPP** — Regulated Pure Pursuit
  - **VPC** — Vector Pursuit Controller

- 📊 **Визуализация**
  - 3D модель пикапа
  - Skybox
  - Costmap
  - Траектории MPPI и State Lattice
  - ImGui окно настройки

## Управление

| Клавиша | Действие |
|---------|----------|
| **W/A/S/D** | Газ / Влево / Тормоз / Вправо |
| **Space** | Сброс скорости |
| **Shift** | Сброс поворота |
| **M** | Переключение контроллера |
| **Tab** | Показать/скрыть costmap |
| **F1** | Переключение камеры |
| **F2** | Окно настройки подвески |
| **R** | Сброс автомобиля |
| **L** | Переключение визуализации |
| **ПКМ** | Вращение камеры |
| **ЛКМ (Costmap)** | Установка цели |

## Сборка

```bash
mkdir build && cd build
qmake ../Sim2.pro
make -j$(nproc)

## Требования

Qt 5.12+

Eigen3

OpenMP

CUDA (для MPPI)

## Структура

Simulation/ — симулятор и физика

MPPI-Generic/ — MPPI контроллер

nav2_smac_planner/ — State Lattice планировщик

Regulated-Pure-Pursuit/ — RPP контроллер

Vector-Pursuit-Controller/ — VPC контроллер
