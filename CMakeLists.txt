# ILI9341

set( srcs "ili9341.c" )
set( include_dirs "." )
set( priv_include_dirs )
set( requires "fonts hal_esp32" )
set( priv_requires )

idf_component_register(
	SRCS ${srcs}
	INCLUDE_DIRS ${include_dirs}
	PRIV_INCLUDE_DIRS ${priv_include_dirs}
	REQUIRES ${requires}
	PRIV_REQUIRES ${priv_requires}
)
