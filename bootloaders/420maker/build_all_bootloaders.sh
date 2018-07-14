#!/bin/bash -ex

BOARD_ID=industruino_420maker NAME=saml21_sam_ba_industruino_420maker MCU=SAML21E18B SAM_BA_INTERFACE=SAM_BA_USBCDC_ONLY make clean all

BOARD_ID=arduino_zero NAME=samd21_sam_ba make clean all

BOARD_ID=genuino_zero NAME=samd21_sam_ba_genuino make clean all

echo Done building bootloaders!
