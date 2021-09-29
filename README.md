# MiAPR_Project
Implementacja algorytmu RRT do planowania ruchu pojazdu o kinematyce samochodowej, wykorzystując krzywe Dubinsa, na mapie zajętości. 

Model samochodu:

![](car_model.png)

o dynamice

```python
x[t+1]     = x[t]     + cos(theta[t])
y[t+1]     = y[t]     + sin(theta[t])
theta[t+1] = theta[t] + tan(phi[t]) / L
```

zmienne:
 - `x`: pozycja pozioma
 - `y`: pozycja pionowa
 - `theta`: kąt kursu (kierunek jazdy)

Zmienna, którą sterujemy
 - `phi ∈ [-pi/5, pi/5]`:kąt skrętu (w stosunku do kierunku jazdy).

Użycie
```bash
$ python3 rrt.py
```

W pliku `TestParameters/cases.py` można zdefiniować wygląd środowiska oraz oczekiwane pozycje początkową oraz końcową

Grupa: 
- Zuzanna Stolc
- Paweł Woźniak
- Łukasz Kozak