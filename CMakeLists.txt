idf_component_register(
	SRCS "ili9341.c"
	INCLUDE_DIRS "."
	PRIV_REQUIRES fonts hal_esp32 printfx systiming
)
