# Подключаем индикацию питания и BLE-статуса только для прошивок клавиатуры/донгла (не трогая settings_reset)
target_sources_ifdef(CONFIG_ZMK_SPLIT app PRIVATE src/status_led.c)

# Аппаратный процессор динамической акселерации трекбола для прошивок с поддержкой pointing
target_sources_ifdef(CONFIG_ZMK_POINTING app PRIVATE src/input_processor_accel.c)

if(CONFIG_ZMK_POINTING)
    target_include_directories(app PRIVATE ${APPLICATION_SOURCE_DIR}/include)
endif()
