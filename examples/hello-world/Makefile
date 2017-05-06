# name of your application
APPLICATION = hello-world

# If no BOARD is found in the environment, use this default:
BOARD ?= native

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

FEATURES_REQUIRED += periph_timer
USEMODULE += xtimer

FEATURES_REQUIRED += periph_spi
FEATURES_REQUIRED += periph_i2c

# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
#USEMODULE += gnrc_netdev_default
#USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
#USEMODULE += gnrc_ipv6_router_default
#USEMODULE += gnrc_udp
# Add a routing protocol
#USEMODULE += gnrc_rpl
#USEMODULE += auto_init_gnrc_rpl
# This application dumps received packets to STDIO using the pktdump module
#USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
#USEMODULE += gnrc_icmpv6_echo
# Add also the shell, some shell commands
#USEMODULE += shell
#USEMODULE += shell_commands
#USEMODULE += ps
#USEMODULE += netstats_l2
#USEMODULE += netstats_ipv6
#USEMODULE += netstats_rpl

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

DEFAULT_CHANNEL ?= 5
FLAGS += -DIEEE802154_DEFAULT_SUBGHZ_CHANNEL=$(DEFAULT_CHANNEL)

include $(RIOTBASE)/Makefile.include
