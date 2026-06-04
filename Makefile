.PHONY: test test-verbose test-filter clean

# Run all tests in the native environment
test:
	pio test -e native

# Run with verbose output (shows individual test results)
test-verbose:
	pio test -e native -v

# Run a specific test filter, e.g.: make test-filter FILTER=DoorAuth
test-filter:
	pio test -e native -f "$(FILTER)"

clean:
	pio run -e native -t clean
