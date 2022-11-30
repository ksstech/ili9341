# ILI9341

set( srcs "ili9341.c" )
set( include_dirs "." )
set( include_dirs "." )
set( requires common )
set( priv_requires driver fonts hal_esp32 printfx syslog systiming )

idf_component_register(
	SRCS ${srcs} 
	INCLUDE_DIRS ${include_dirs} 
	PRIV_INCLUDE_DIRS ${priv_include_dirs} 
	REQUIRES ${requires} 
	PRIV_REQUIRES ${priv_requires}
)
