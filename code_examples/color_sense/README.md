# Программа для работы с датчиком цвета TZT TCS34725

Сборка:  
```
gcc color_sense.c -o color_sense -Wall  
```

Запуск:  
sudo ./color_sense [-h][-q]
-h - описание работы
-q - тихий режим
В тихом режиме вывод производится в формате:
```
code: R G B
```
где R - интенсивность красного цвета, G - интенсивность зеленого цвета, B - интенсивность синего цвета,