# Программа для работы с RFID считывателем

Сборка:  
``` 
g++ MFRC522.cpp rfid_reader.cpp -std=c++11 -lbcm2835 -o rfid_reader   
``` 

Для корректной работы устройства необходимо установить стороннюю библиотеку:  
```  
  tar zxvf bcm2835-1.70.tar.gz  
  cd bcm2835-1.70  
  ./configure  
  make  
  sudo make check  
  sudo make install  
```    

Пример работы: 
```   
 sudo ./rfid_reader
```  
Enter your choice:  
        1: Print UID  
        2: Write information  
        3: Read information  

1 - вывод на экран значения UID  
2 - запись информации в блок  
3 - чтение информации из блока  
