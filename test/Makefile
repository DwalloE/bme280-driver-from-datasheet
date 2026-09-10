# Host-native tests for main/bme280.c. Needs only cc and make; ~1 s.
#
#   make          - everything: functional suite, then the coverage gate
#
# Evidence order, as in project 02: stage 0 of the suite first proves the
# harness can catch a driver that ignores bus errors (the naive control
# must wrongly succeed against a stuck bus and an empty bus), and only
# then does the real driver's clean run count for anything.
#
# No ThreadSanitizer stage here on purpose: the driver is single-context
# by design (one caller, no ISR side), so there is no concurrency claim
# to certify. Cargo-culting 02's TSan stage would test nothing.
#
# The gate: 100% branch coverage on bme280.c. The error model is the
# product, and an untaken error branch is exactly what the gate exists to
# catch. Per 02's lesson, the instrumented object is compiled INSIDE
# build/ (-c, then link) so Apple and GNU gcov agree on where the
# .gcno/.gcda land and the gate works on macOS too.

CC     ?= cc
CFLAGS ?= -O2 -Wall -Wextra -Werror -std=c11

BUILD := build

.PHONY: all run functional coverage clean

all: run

run: functional coverage
	@echo
	@echo "ALL TEST STAGES PASS"

$(BUILD):
	@mkdir -p $(BUILD)

# ---------------------------------------------------------------- functional
$(BUILD)/test_bme280: test_bme280.c mock_bus.c ../main/bme280.c \
                      ../main/bme280.h mock_bus.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ test_bme280.c mock_bus.c ../main/bme280.c -lm

functional: $(BUILD)/test_bme280
	@echo "--- functional: controls, calibration traps, worked example, fault sweep"
	./$(BUILD)/test_bme280

# ------------------------------------------------------------------- coverage
# Separate -O0 --coverage build so gcov's branch view matches the source.
# Only bme280.c is gated; the mock and the tests are scaffolding.
coverage: | $(BUILD)
	@echo "--- coverage: every branch in bme280.c must be taken both ways"
	cd $(BUILD) \
	    && $(CC) -O0 --coverage -std=c11 -Wall -Wextra -Werror \
	           -c ../../main/bme280.c -o cov_bme280.o \
	    && $(CC) -O0 -std=c11 -Wall -Wextra -Werror \
	           -c ../test_bme280.c -o cov_test.o \
	    && $(CC) -O0 -std=c11 -Wall -Wextra -Werror \
	           -c ../mock_bus.c -o cov_mock.o \
	    && $(CC) --coverage cov_bme280.o cov_test.o cov_mock.o -o cov_test \
	           -lm \
	    && ./cov_test > /dev/null \
	    && gcov -b cov_bme280.gcda > gcov_summary.txt
	@awk -v RS='' '/bme280\.c/ { found = 1;                                   \
	        if ($$0 ~ /Taken at least once:100\.00%/) ok = 1;                  \
	        print }                                                            \
	    END {                                                                  \
	        if (!found) { print "no gcov stanza for bme280.c - if this is";    \
	            print "macOS gcov formatting, the Linux CI run is the";        \
	            print "authoritative check"; exit 2 }                          \
	        exit ok ? 0 : 1 }' $(BUILD)/gcov_summary.txt

clean:
	rm -rf $(BUILD) *.gcno *.gcda *.gcov
