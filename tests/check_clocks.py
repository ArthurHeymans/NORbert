#!/usr/bin/env python3
"""Check this board's static PLL/SDC contract, not physical timing closure."""

import argparse
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise ValueError(message)


def uncomment(text):
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"(?m)(?://|#).*?$", "", text)


def pll_value(pll, name):
    values = re.findall(rf"\bdefparam\s+pll_inst\.{re.escape(name)}\s*=\s*([^;]+);", uncomment(pll))
    require(len(values) == 1, f"PLL parameter {name}: expected one definition")
    return values[0].strip().strip('"')


def number(value):
    result = float(value)
    require(math.isfinite(result), f"non-finite clock value: {value}")
    return result


def clock(sdc, name):
    lines = re.findall(rf"(?m)^create_(?:generated_)?clock\s+-name\s+{re.escape(name)}\s+(.+)$", uncomment(sdc))
    require(len(lines) == 1, f"SDC clock {name}: expected one declaration")
    return lines[0]


def option(line, name, default=None):
    values = re.findall(rf"(?:^|\s)-{re.escape(name)}\s+([^\s\[\]]+)", line)
    if not values and default is not None:
        return default
    require(len(values) == 1, f"SDC option -{name}: expected one value")
    return number(values[0])


def close(actual, expected, label, tolerance=0.001):
    require(abs(actual - expected) <= tolerance, f"{label}: got {actual:g}, expected {expected:g}")


def check_sources(pll, vendor, oss):
    """Support the checked-in integer, static, internal-feedback PLL profile."""
    require(pll_value(pll, "CLKFB_SEL") == "INTERNAL", "only internal PLL feedback is checked")
    for name in ("MDIV_FRAC_SEL", "ODIV0_FRAC_SEL"):
        close(number(pll_value(pll, name)), 0, name)
    for name in ("DYN_DPA_EN", "SSC_EN", "DYN_PE0_SEL", "DYN_PE1_SEL", "DYN_PE2_SEL"):
        require(pll_value(pll, name) == "FALSE", f"dynamic PLL setting {name} needs separate review")
    close(number(pll_value(pll, "CLKOUT0_PE_COARSE")), 0, "main PLL coarse phase")
    close(number(pll_value(pll, "CLKOUT0_PE_FINE")), 0, "main PLL fine phase")

    input_mhz = number(pll_value(pll, "FCLKIN"))
    idiv = number(pll_value(pll, "IDIV_SEL"))
    mdiv = number(pll_value(pll, "MDIV_SEL"))
    require(input_mhz > 0 and idiv > 0 and mdiv > 0, "PLL frequencies/dividers must be positive")
    input_period = 1000 / input_mhz
    close(option(clock(vendor, "osc_clk"), "period"), input_period, "oscillator period")
    require("[get_ports {clk_50mhz}]" in clock(vendor, "osc_clk"), "oscillator must bind clk_50mhz")

    for channel, vendor_name, oss_name, endpoint in (
        (0, "sys_clk", "clk", "[get_nets {clk_133}]"),
        (1, "sdram_clk", "sdram_clk", "[get_ports {O_sdram_clk}]"),
        (2, "aux_clk", "aux_clk", "[get_nets {aux_clk}]"),
    ):
        require(pll_value(pll, f"CLKOUT{channel}_EN") == "TRUE", f"PLL output {channel} must be enabled")
        odiv = number(pll_value(pll, f"ODIV{channel}_SEL"))
        require(odiv > 0, "PLL output divider must be positive")
        coarse = number(pll_value(pll, f"CLKOUT{channel}_PE_COARSE"))
        fine = number(pll_value(pll, f"CLKOUT{channel}_PE_FINE"))
        # UG306-1.0.4E section 5.1, p46: a coarse step is a FULL VCO period.
        phase = ((coarse + fine / 8) / odiv * 360) % 360
        period = input_period * idiv * odiv / mdiv
        line = clock(vendor, vendor_name)
        require("[get_ports {clk_50mhz}]" in line and endpoint in line, f"{vendor_name}: wrong source/endpoint")
        declared_period = input_period * option(line, "divide_by") / option(line, "multiply_by")
        close(declared_period, period, f"{vendor_name} period")
        close(option(line, "phase", 0), phase, f"{vendor_name} phase", 0.000001)
        close(option(clock(oss, oss_name), "period"), period, f"OSS {oss_name} period")

    for sdc in (vendor, oss):
        require("[get_ports {spi_clk_pin}]" in clock(sdc, "spi_clk"), "SPI clock must bind input port spi_clk_pin")
    spi_period = option(clock(vendor, "spi_clk"), "period")
    require(spi_period > 0, "SPI period must be positive")
    close(option(clock(oss, "spi_clk"), "period"), spi_period, "OSS/vendor SPI periods")


def check_report(report, oss, log):
    """Require actual packed clock targets; failed experimental Fmax is allowed.

    These are the packed clock aliases emitted by the pinned NORbert flow.
    Missing/renamed clocks fail closed rather than accepting an arbitrary clock
    with the same frequency. Aux/forwarded outputs have no same-domain Fmax
    entry: this does not claim their phase or I/O timing has been analyzed.
    """
    require(not re.search(r"get_(?:nets|ports).*matched no objects", log), "nextpnr: unmatched clock constraint")
    for net, name in (("clk", "clk"), ("glue_i.spi_clk", "spi_clk")):
        entry = report.get("fmax", {}).get(net)
        require(isinstance(entry, dict) and "constraint" in entry, f"nextpnr: missing required clock {net}")
        expected = 1000 / option(clock(oss, name), "period")
        close(number(entry["constraint"]), expected, f"nextpnr {net} target MHz", 0.02)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--oss-sdc", type=Path, default=Path("tangprimer25k.oss.sdc"))
    parser.add_argument("--report", type=Path)
    parser.add_argument("--log", type=Path)
    args = parser.parse_args()
    oss = (args.root / args.oss_sdc).read_text()
    try:
        check_sources((args.root / "src/pll.v").read_text(), (args.root / "tangprimer25k.sdc").read_text(), oss)
        if args.report is not None:
            require(args.log is not None, "--report requires --log")
            check_report(json.loads(args.report.read_text()), oss, args.log.read_text())
        print("Clock contract OK (not timing sign-off)")
    except (ValueError, TypeError, KeyError) as error:
        parser.exit(1, f"Clock contract failed: {error}\n")


if __name__ == "__main__":
    main()
