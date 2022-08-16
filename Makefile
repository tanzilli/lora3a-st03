APPLICATION = default
RIOTBASE ?= $(CURDIR)/../RIOT
EXTERNAL_BOARD_DIRS ?= $(CURDIR)/../lora3a-boards/boards
BOARD ?= lora3a-st02

USEMODULE += xtimer
USEMODULE += periph_i2c
USEMODULE += periph_uart
USEMODULE += periph_uart_modecfg
USEMODULE += ztimer_usec
USEMODULE += printf_float
USEMODULE += shell
USEMODULE += periph_wdt
USEMODULE += periph_flashpage

QUIET ?= 1
DEVELHELP ?= 1

ADDRESS ?= 1

include $(RIOTBASE)/Makefile.include



