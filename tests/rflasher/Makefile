NORBERT_RTL_ROOT ?= ../..
RTL := $(abspath $(NORBERT_RTL_ROOT))
HERE := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
BUILD ?= $(HERE)build
SOURCES := $(addprefix $(RTL)/src/,spi_trx.v spi_prefetch.v glue.v spi_program.v host_protocol.v chip_profile_store.v sdram_client_mux.v)

.PHONY: test-rflasher bridge clean
bridge: $(BUILD)/Vwrapper
$(BUILD)/Vwrapper: $(HERE)wrapper.v $(HERE)bridge.cpp $(SOURCES) $(wildcard $(RTL)/src/*.vh)
	mkdir -p $(BUILD)
	verilator --cc --exe --build -j 2 --top-module wrapper -I$(RTL)/src \
		-Wno-CASEINCOMPLETE -Wno-PINMISSING --Mdir $(BUILD) \
		-CFLAGS '-std=c++17' $(HERE)wrapper.v $(SOURCES) $(HERE)bridge.cpp

test-rflasher: bridge
	NORBERT_BRIDGE=$(BUILD)/Vwrapper cargo test --locked --manifest-path $(HERE)Cargo.toml -- --test-threads=1
clean:
	rm -rf $(BUILD)
