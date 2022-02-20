Multi-Objective optimization
---
This is a small implementation of two algorithms used to find Pareto optimal front.
The implementation can only handle two objective functions and two variables for visualization purpose.

Prerequisites
---
In order to run this program you'll need to download an optimization solver.
We used the "Baron" solver as it can solve both linear and non-linear optimization problems - [Download Link](https://minlp.com/baron-downloads).

Install the program and save to a directory (default path is "C:\baron").

Run
---
```
git clone https://github.com/shaigundersen/Optimization_Project.git
cd /path/to/dir
pip3 install -r requirements.txt
python3 multiobjective_optimization.py solver_name path_to_solver.exe 
```
for example: `python3 multiobjective_optimization.py baron C:\baron\baron.exe`

Cones Problem definition
---
We would like to construct a cone such that both it's lateral surface and total surface are minimized.
The base radius and height of the cone should not exceed 10 and 20 cm respectively.
Each cone should have a volume of at least 200 cm^3

**Notations**

r = base radius, h = height, s = slant height V = volume, B = base area, S = lateral surface area ,T = total area

**Matematical problem modeling**

- <img src="https://latex.codecogs.com/svg.image?min&space;(S)=\pi*r*s" title="min S=\pi*r*s" />
- <img src="https://latex.codecogs.com/svg.image?min&space;(T)&space;=&space;B&space;&plus;&space;S&space;=&space;\pi*r*(r&plus;s)" title="min T = B + S = \pi*r*(r+s)" />
such that:
- <img src="https://latex.codecogs.com/svg.image?V\geq&space;200cm^3" title="V\geq 200cm^3" />
- <img src="https://latex.codecogs.com/svg.image?r\in[0,10]cm,&space;h\in[0,20]cm" title="r\in[0,10]cm, h\in[0,20]cm" />
<img src="https://www.varsitytutors.com/assets/vt-hotmath-legacy/hotmath_help/topics/volume-of-a-cone/cone1.gif" />
