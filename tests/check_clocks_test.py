import copy
from pathlib import Path
import unittest

from check_clocks import check_report, check_sources

ROOT = Path(__file__).resolve().parents[1]


class ClockContractTests(unittest.TestCase):
    def setUp(self):
        self.pll = (ROOT / "src/pll.v").read_text()
        self.vendor = (ROOT / "tangprimer25k.sdc").read_text()
        self.oss = (ROOT / "tangprimer25k.oss.sdc").read_text()
        # Independent targets, not frequencies taken from the report under test.
        period = "14.286" if "14.286" in self.oss else "25.000"
        self.report = {"fmax": {
            "clk": {"constraint": 120.0, "achieved": 90.0},
            "glue_i.spi_clk": {"constraint": 1000 / float(period), "achieved": 60.0},
        }}

    def test_checked_in_contract(self):
        check_sources(self.pll, self.vendor, self.oss)

    def test_both_stack_spi_targets(self):
        for period in ("25.000", "14.286"):
            with self.subTest(period=period):
                vendor = self.vendor.replace("25.000", period).replace("14.286", period)
                oss = self.oss.replace("25.000", period).replace("14.286", period)
                check_sources(self.pll, vendor, oss)

    def test_half_vco_sdram_phase_rejected(self):
        with self.assertRaisesRegex(ValueError, "sdram_clk phase"):
            check_sources(self.pll, self.vendor.replace("-phase 324", "-phase 162"), self.oss)

    def test_half_vco_aux_phase_rejected(self):
        with self.assertRaisesRegex(ValueError, "aux_clk phase"):
            check_sources(self.pll, self.vendor.replace("-phase 180", "-phase 90"), self.oss)

    def test_stale_pll_phase_rejected(self):
        pll = self.pll.replace("CLKOUT2_PE_COARSE = 5;", "CLKOUT2_PE_COARSE = 6;")
        with self.assertRaisesRegex(ValueError, "aux_clk phase"):
            check_sources(pll, self.vendor, self.oss)

    def test_fine_phase_uses_eighth_of_full_vco_period(self):
        pll = self.pll.replace("CLKOUT2_PE_FINE = 0;", "CLKOUT2_PE_FINE = 4;")
        check_sources(pll, self.vendor.replace("-phase 180", "-phase 198"), self.oss)

    def test_frequency_drift_rejected(self):
        with self.assertRaisesRegex(ValueError, "sys_clk period"):
            check_sources(self.pll.replace("MDIV_SEL = 24;", "MDIV_SEL = 25;"), self.vendor, self.oss)

    def test_dynamic_phase_requires_review(self):
        with self.assertRaisesRegex(ValueError, "dynamic PLL setting"):
            check_sources(self.pll.replace('DYN_PE2_SEL = "FALSE";', 'DYN_PE2_SEL = "TRUE";'), self.vendor, self.oss)

    def test_deleted_or_duplicate_declaration_rejected(self):
        spi = next(line for line in self.oss.splitlines() if line.startswith("create_clock -name spi_clk "))
        for oss in (self.oss.replace(spi, ""), self.oss + "\n" + spi):
            with self.subTest(oss=oss), self.assertRaisesRegex(ValueError, "expected one declaration"):
                check_sources(self.pll, self.vendor, oss)

    def test_old_spi_net_selector_rejected(self):
        with self.assertRaisesRegex(ValueError, "SPI clock must bind input port"):
            check_sources(self.pll, self.vendor, self.oss.replace("[get_ports {spi_clk_pin}]", "[get_nets {spi_clk_in}]"))

    def test_oss_period_drift_rejected(self):
        with self.assertRaisesRegex(ValueError, "OSS aux_clk period"):
            check_sources(self.pll, self.vendor, self.oss.replace("-name aux_clk -period 8.333", "-name aux_clk -period 10.000"))

    def test_report_checks_target_not_achieved_frequency(self):
        # Experimental timing failures are not silently promoted to sign-off,
        # but do not prevent building a functional test bitstream.
        check_report(self.report, self.oss, "")

    def test_unconstrained_12mhz_spi_rejected(self):
        self.report["fmax"]["glue_i.spi_clk"]["constraint"] = 12.0
        with self.assertRaisesRegex(ValueError, "target MHz"):
            check_report(self.report, self.oss, "")

    def test_missing_or_renamed_clocks_rejected(self):
        for net in ("clk", "glue_i.spi_clk"):
            with self.subTest(net=net):
                report = copy.deepcopy(self.report)
                report["fmax"]["another_clock"] = report["fmax"].pop(net)
                with self.assertRaisesRegex(ValueError, "missing required clock"):
                    check_report(report, self.oss, "")

    def test_missing_report_rejected(self):
        with self.assertRaisesRegex(ValueError, "missing required clock"):
            check_report({}, self.oss, "")

    def test_nonfinite_target_rejected(self):
        self.report["fmax"]["clk"]["constraint"] = float("nan")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            check_report(self.report, self.oss, "")

    def test_unmatched_clock_warning_rejected(self):
        for selector in ("get_nets", "get_ports"):
            with self.subTest(selector=selector), self.assertRaisesRegex(ValueError, "unmatched clock constraint"):
                check_report(self.report, self.oss, f"Warning: {selector} argument 'typo' matched no objects.")


if __name__ == "__main__":
    unittest.main()
