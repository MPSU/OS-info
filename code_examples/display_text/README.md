# Программа для работы с дисплеем

Сборка:
```  
gcc -o lcd main.c lcd.c -lgd -lfreetype -lm  
```

Запуск:  
```
sudo ./lcd [text]  
``` 

Выходные данные: 
Печать заданного текста на дисплее  

Пример работы: 

```
sudo ./lcd test_string # печать текста test_string
sudo ./lcd "text with spaces" # печать текста с пробелами
echo "text with spaces" | xargs -I {} sudo ./lcd "{}" # печать вывода другого приложения
```

Для корректной сборки программы необходимо установить приложенную библиотеку libgd-2.3.3  
Для этого требуется выполнить следующие действия:  
```
tar -xvf libgd-2.3.3.tar.gz
cd libgd-2.3.3  
chmod +x configure  
./configure  
make  
sudo make install  
```

