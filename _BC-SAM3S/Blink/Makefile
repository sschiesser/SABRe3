APP_NAME = Blink
PROJECT_NAME = DERFUSB23E06
CONFIG_NAME = All_deRFusb23e06_At91sam3s4c_Rf231_Gcc

#PROJECT_NAME = RF231USBRD
#CONFIG_NAME = All_Rf231UsbRd_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_Sec_Rf231UsbRd_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_StdlinkSec_Rf231UsbRd_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_Rf231UsbRd_At91sam3s4c_Rf231_Iar
#CONFIG_NAME = All_Sec_Rf231UsbRd_At91sam3s4c_Rf231_Iar
#CONFIG_NAME = All_StdlinkSec_Rf231UsbRd_At91sam3s4c_Rf231_Iar

#PROJECT_NAME = SAM3S_EK
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf230B_Gcc
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf212_Gcc
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf230B_Gcc
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf212_Gcc
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf230B_Gcc
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf231_Gcc
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf212_Gcc
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf230B_Iar
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf231_Iar
#CONFIG_NAME = All_Sam3sEk_At91sam3s4c_Rf212_Iar
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf230B_Iar
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf231_Iar
#CONFIG_NAME = All_Sec_Sam3sEk_At91sam3s4c_Rf212_Iar
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf230B_Iar
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf231_Iar
#CONFIG_NAME = All_StdlinkSec_Sam3sEk_At91sam3s4c_Rf212_Iar

all:
	make -C makefiles/$(PROJECT_NAME) -f Makefile_$(CONFIG_NAME) all APP_NAME=$(APP_NAME)

clean:
	make -C makefiles/$(PROJECT_NAME) -f Makefile_$(CONFIG_NAME) clean APP_NAME=$(APP_NAME)
