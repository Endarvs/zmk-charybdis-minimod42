# Подключаем индикацию питания и BLE-статуса только для прошивок клавиатуры/донгла (не трогая settings_reset)
target_sources_ifdef(CONFIG_ZMK_SPLIT app PRIVATE src/status_led.c)

# Живой процессор скорости, динамической акселерации, скролла, снайпера и Auto-Mouse
target_sources_ifdef(CONFIG_ZMK_POINTING app PRIVATE src/input_processor_accel.c)

# Второй виртуальный USB CDC-ACM порт (MI_02) + NVS-память и опрос батарей половин для Charybdis Helper
# (активируется автоматически только при наличии узла helper_cdc_acm_uart в overlay)
target_sources(app PRIVATE src/helper_bridge.c)
target_include_directories(app PRIVATE ${APPLICATION_SOURCE_DIR}/include)
