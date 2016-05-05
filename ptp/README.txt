This PTP package contains everything related to Marvell PTP hardware
from low HW-level up to high service level.

mv_ptp_if.o      - low level ptp/tai configuration and access procedures
mv_tai_clock.o   - low level tai-clock configuration and access procedures
mv_ptp_if_serv.o - service interfacing to sysfs
mv_ptp_sysfs.o   - sysfs commands run-time used by User-Space PTP application
mv_ptp_uio.o     - uio device driver mapping all tai/ptp registers for
                            User-Space PTP application
mv_pp2x_ptp_hook.c - include/hook into mvppXX network RX/TX packet device

The PTP package is a built-in extension of generic Network-device mvppXX.
It is wrapped with compilation flag CONFIG_MV_PTP_SERVICE (enabled by
Makefile_ptp).

To build mvppXX with PTP extension copy Makefile_ptp into mvppXX/Makefile
or in mvppXX directory make link:   ln -s ptp/Makefile Makefile

