## Этап 7 проекта В07

__Задание:__  
1. Скачайте с репозитория исходные коды программ `rangefinder_hcsr04_ФИО`, `play_note_ФИО`, `combiner`, make файлы и bash-скрипт для их запуска, скопируйте их в соответсвующие поддиректории папки `./stage_07`.
2. Модернизируйте исходный код драйвера GPIO (работа с кнопкой, был разработан на 7-ой Л.Р.): использовать вывод GPIO11, подключенный к выводу ECHO первого дальномера HC-SR01, регистрировать прерывания как по переднему фронту GPIO11, так и по заднему фронту.
3. Модернизируйте программу `rangefinder_hcsr04_ФИО`, которая будет работать с первым дальномером, добавив в нее работу с драйвером GPIO и обработку прерываний от вывода ECHO (по переднему и заднему фронту). 
4. Модернизируйте bash-скрипт таким образом, чтобы разработанный драйвер автоматически подгружался при включении RPi.
5. Добавьте bash-скрипт в автозагрузку RPi и убедитесь, что все программы запускаются должным образом при включении RPi.
6. Скопируйте на ПК исходные коды, make файлы и bash-скрипт из `stage_07`. Добавьте новое readme.md с иснтрукцией по запуску программ и сделайте коммит на сервер.
7. Продемонстрируйте преподавателю работу программ и bash-скрипта, а также созданный репозиторий. 
8. Подготовьте ответы на вопросы к лабораторной работе.

__Список вопросов:__  
1. Какие заголовочные файлы Вы добавили в проект, при модернизации программы `sound_detect_ФИО`?
2. С помощью каких функций осуществляется инциализация модуля драйвера?
3. Каким образом Вы реализовали автозагрузку модуля драйвера при включении RPi?
4. Какие еще есть способы загрузки модулей?
5. Изменилась ли процедура взаимодействия других программ (`light_detect_ФИО`, `combiner`) с драйвером?
6. Использовался ли механизм обработки прерываний модуля и почему?
7. К какому классу драйвера устройств относится Ваш драйвер?
8. Какие этапы разработки Вашего драйвера являются обязательными?


__Порядок выполнения и сдачи [курсового проекта](var_07_task.md):__
1. [Этап проекта №1](var_07_stage_01.md)
2. [Этап проекта №2](var_07_stage_02.md)
3. [Этап проекта №3](var_07_stage_03.md)
4. [Этап проекта №4](var_07_stage_04.md)
5. [Этап проекта №5](var_07_stage_05.md)
6. [Этап проекта №6](var_07_stage_06.md)
7. [Этап проекта №7](var_07_stage_07.md)
8. [Этап проекта №8](var_07_stage_08.md)
