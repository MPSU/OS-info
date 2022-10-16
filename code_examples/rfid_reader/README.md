# Программа для работы с RFID считывателем

Сборка:  
``` 
gcc MFRC522.c spi.c gpio.c rfid_reader.c -o rfid_reader   
```  
Запуск:  
./rfid_reader [-h][-q]  
-h - описание работы  
-q - тихий режим 

Пример работы: 
```   
 sudo ./rfid_reader
```  
```
Card detected
01 02 03 04
```