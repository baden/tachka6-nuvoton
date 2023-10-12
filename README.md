## Намагаюсь запутити стару плату NUC140VE3CN

Знадобилась плата на 5В. Знайшлась стара демо плата з NUC140VE3CN.
Задача ускладнилась тим, шо я раніше викусив програматор і шукали його якось влом.
Спробую записати через WeAct Studio Mini Debugger.

`pyocd list` його бачить.
`pyocd list --target` показує підтримку nuc140ve3cn

Це мабуть тому шо я до цього якось (не памʼятаю як) завантажив шось для підтримки Nuvoton.

До речі openocd теж, здається, підтримує цей процесор. Буде план "Б".

Шукаємо BSP.
Чи це воно? https://github.com/OpenNuvoton/NUC100BSP

```
git clone https://github.com/OpenNuvoton/NUC100BSP.git
```

Знайшов даташит
https://www.nuvoton.com/export/resource-files/NuTiny-SDK-NUC140_User_Manual_EN_V1.01.pdf

Почнемо зі світлодіода.
pin10  I2C1_SDA/PA.10  - Вбудован в плату.

pin32 - PB0 - UART RXD  - JP5, pin7
pin33 - PB1 - UART TXD  - JP5, pin8

pin12 - PA8 - I2C0 SDA
pin11 - PA9 - I2C0 SCL

Херассе. Пришив. Працює!!!!

printf не працює