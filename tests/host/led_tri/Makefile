CC ?= cc
CFLAGS ?= -O2 -std=c11 -Wall -Wextra -Werror
FLAGS = $(CFLAGS) -Wno-unused-parameter -Wno-error=cpp -I.
BINS = test_led_tri test_led_tri_sync
DEPS = test_led_tri.c globals.h \
	../../../src/system/led.c ../../../src/system/led.h \
	../../../src/system/led_strip_fade.h ../../../src/system/led_sync.h

.PHONY: check clean

check: $(BINS)
	@set -e; for test in $(BINS); do ./$$test; done

test_led_tri: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=0 -o $@ test_led_tri.c

test_led_tri_sync: $(DEPS)
	$(CC) $(FLAGS) -DCONFIG_LED_NETWORK_SYNC=1 -o $@ test_led_tri.c

clean:
	$(RM) $(BINS)
