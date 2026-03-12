# Selftest app - built by app/build pack.
# Produces binary named "selftest" (see fw.yaml pack vars: out: selftest).

CC ?= gcc
CFLAGS ?= -Wall -O2

selftest: selftest.c
	$(CC) $(CFLAGS) -o selftest selftest.c

clean:
	rm -f selftest

.PHONY: clean
