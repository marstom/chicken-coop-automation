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


compile_coop:
	pio run -e esp32-c3-chicken-coop

compile_coop_upload_and_monitor:
	pio run -e esp32-c3-chicken-coop -t upload
	pio device monitor

compile_relay_project:
	pio run -e esp32-c3-relay-controller-usb

compile_relay_project_and_monitor:
	pio run -e esp32-c3-relay-controller-usb -t upload
	pio device monitor