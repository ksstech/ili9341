# ILI9341
set( srcs "ili9341.c" )
set( include_dirs "." )
set( requires common )
set( priv_requires fonts hal_esp32 printfx systiming )
idf_component_register(SRCS ${srcs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} PRIV_REQUIRES ${priv_requires})
