# Подключаем индикацию питания и BLE-статуса только для прошивок клавиатуры/донгла (не трогая settings_reset)
target_sources_ifdef(CONFIG_ZMK_SPLIT app PRIVATE src/status_led.c)
