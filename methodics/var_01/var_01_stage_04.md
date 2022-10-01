## Этап 4 проекта В01

__Задание:__  
1. Скачайте с репозитория исходные коды программ [sound_detect](../../code_examples/Датчик%20звука%20KY-038) и [light_detect](../../code_examples/Фоторезистор%20GL5516) и скопируйте их в соответсвующие поддиректории папки `./lab04`.
2. Самостоятельно скомпилируйте  программы `sound_detect` и `light_detect` из исходных файлов.
3. Создайте makefile для каждой из программ, переименовав имена исполняемых файлов на `sound_detect_ФИО` и `light_detect_ФИО`. Обязательные команды: all, clean.
4. Модернизируйте исходные коды программ следующим образом - `sound_detect_ФИО` - добавить к сообщению `clap!` время срабатывания хлопка, а также добавить аргументы при вызове - указатель на именованный канал; `light_detect_ФИО` - должна выдавать время срабатывания датчика по определенному ранее уровню срабатывания отсчетов АЦП, который передается программе как дополнительный аргумент. Скомпилирйте и отладьте работу программ.
5. Модернизируйте bash-скрипт, запускающий эти программы, чтобы передавалось корректное число аргументов для программ `sound_detect_ФИО` и `light_detect_ФИО`, не забудьте про именованные каналы `sound_data` и `light_data`.
6. Скопируйте на ПК исходные коды, make файлы и bash-скрипт из `lab04`. Добавьте новое readme.md с инструкцией по сборке и запуску программ и сделайте коммит на сервер.
7. Продемонстрируйте преподавателю работу программ и bash-скрипта, а также созданный репозиторий. 
8. Подготовьте ответы на вопросы лабораторной работы.

__Список вопросов:__
1. Что значат флаги компиляции программ `sound_detect` и `light_detect`?
2. Что делают команды all и clean в Ваших make файлах? Какие еще команды можно реализовать в Make файлах?
3. С помощью какой библиотеки определяется время в модернизированных программах?
4. Важен ли порядок передачи аргументов в программы при запуске и если да, то можно ли реализовать произвольную передачу аргументов?
5. Какого функционала еще не хватает Вашему комплекту ПО для выполнения требований курсового проекта?
6. Что такое объектные файлы и можно ли их удалить для успешного запуска программ `sound_detect_ФИО` и `light_detect_ФИО`?
7. Можно ли переписать make файл таким образом, чтобы он собирал сразу две программы `sound_detect_ФИО` и `light_detect_ФИО`?
8. Чем отличается Ваша инструкция к этой лабораторной работы от предыдущей?
9. Докажите, что Ваши исходные коды программ удовлетворяют [требованиям оформления исходных кодов](https://www.kernel.org/doc/html/v4.10/process/coding-style.html) на нескольких правилах.
10. Какой следующий этап доработки Вашего проекта?

__Порядок выполнения и сдачи [курсового проекта](var_01_task.md):__
1. [Этап проекта №1](var_01_stage_01.md)
2. [Этап проекта №2](var_01_stage_02.md)
3. [Этап проекта №3](var_01_stage_03.md)
4. [Этап проекта №4](var_01_stage_04.md)
5. [Этап проекта №5](var_01_stage_05.md)
6. [Этап проекта №6](var_01_stage_06.md)
7. [Этап проекта №7](var_01_stage_07.md)
8. [Этап проекта №8](var_01_stage_08.md)
