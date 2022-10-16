# Программа для работы с дальномером gp2yo

Сборка:  
```
gcc rangefinder_gp2y.c -o rangefinder_gp2y -lwiringPi -lm -Wall
```

В случае, если отображается "cannot find -lwiringPi", необходимо установить форк WiringPi: https://github.com/WiringPi/WiringPi
Установка:
```
./build
```

Запуск:  
sudo ./rangefinder_gp2y [-h][-q] N  
-h - описание работы  
-q - тихий режим, выводится только расстояние до объекта  
N - период вывода значений в мс  

Пример:  
```
sudo ./rangefinder_gp2y -q 1000 //Вывод значений один раз в секунду в "тихом режиме"