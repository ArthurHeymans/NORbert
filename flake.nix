{
  description = "Tang Primer 25K SPI Flash Emulator";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    gowin-eda = {
      url = "github:Blue-Berry/gowin-eda.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      rust-overlay,
      gowin-eda,
    }:
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ (import rust-overlay) ];
          config.allowUnfree = true;
        };

        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [
            "rust-src"
            "rust-analyzer"
            "rustfmt"
            "clippy"
          ];
          targets = [ "wasm32-unknown-unknown" ];
        };

        # Gowin EDA Education edition from gowin-eda.nix
        gowinEda = gowin-eda.packages.${system}.default;

        # Keep the CLI exactly in sync with the wasm-bindgen crate in
        # tool/Cargo.lock. Mismatched versions cannot process its Wasm output.
        wasmBindgenCli = pkgs.runCommand "wasm-bindgen-cli-0.2.127" { } ''
          mkdir -p $out/bin
          install -m755 ${
            pkgs.fetchzip {
              url = "https://github.com/wasm-bindgen/wasm-bindgen/releases/download/0.2.127/wasm-bindgen-0.2.127-x86_64-unknown-linux-musl.tar.gz";
              hash = "sha256-X6rT4cwbJaCX/oyWX8NDN4u6uUS/Io5axRy8nvUQi7I=";
            }
          }/{wasm-bindgen,wasm-bindgen-test-runner,wasm2es6js} $out/bin/
        '';

        # Create a gw_sh wrapper using the same FHS approach
        gowinSrc = pkgs.stdenv.mkDerivation {
          name = "gowin-eda-edu-src";
          src = pkgs.fetchurl {
            url = "https://cdn.gowinsemi.com.cn/Gowin_V1.9.11.03_Education_Linux.tar.gz";
            hash = "sha256-b9OS90c7JNhHtvjr3HoYXFkYJro12NDlF5YQMNRG+fc=";
          };
          sourceRoot = ".";
          installPhase = ''
            mkdir -p $out
            cp -r * $out/
            rm -f $out/IDE/lib/libfreetype.so* || true
          '';
        };

        gowinFhs = pkgs.buildFHSEnv {
          name = "gowin-fhs";
          targetPkgs =
            pkgs: with pkgs; [
              stdenv.cc.cc.lib
              xorg.libX11
              xorg.libXext
              xorg.libXrender
              xorg.libXtst
              xorg.libXi
              xorg.libXrandr
              xorg.libXcomposite
              xorg.libXcursor
              xorg.libXdamage
              xorg.libXfixes
              xorg.libXScrnSaver
              xorg.libxcb
              xorg.xcbutil
              xorg.xcbutilimage
              xorg.xcbutilkeysyms
              xorg.xcbutilrenderutil
              xorg.xcbutilwm
              libGL
              libGLU
              libdrm
              mesa
              fontconfig
              freetype
              libpng12
              glib
              zlib
              dbus
              expat
              libuuid
              systemd
              alsa-lib
              libpulseaudio
              nss
              nspr
              krb5
              cups
              wayland
              libxkbcommon
              qt5.qtbase
              qt5.qtmultimedia
              qt5.qtserialport
              qt5.qtwayland
              gst_all_1.gstreamer
              gst_all_1.gst-plugins-base
              bash
              coreutils
            ];
          runScript = "bash";
        };

        # Wrapper script for gw_sh (command-line synthesis)
        gwSh = pkgs.writeShellScriptBin "gw_sh" ''
          set -e
          WORKSPACE_DIR="$HOME/.local/share/gowin-eda-edu"

          if [ ! -d "$WORKSPACE_DIR/IDE" ]; then
            echo "First run: Setting up Gowin IDE workspace..."
            mkdir -p "$WORKSPACE_DIR"
            cp -r ${gowinSrc}/* "$WORKSPACE_DIR/"
            chmod -R u+w "$WORKSPACE_DIR"
            echo "Setup complete!"
          fi

          # Offscreen mode: unset DISPLAY and use Qt offscreen platform to
          # avoid GLX initialization failures in headless/server environments.
          # GOWIN's PnR tool internally uses Qt OpenGL widgets, which fail
          # with "Could not initialize GLX" if a DISPLAY is set without a
          # working GLX implementation (e.g. over Xvfb without Mesa swrast).
          unset DISPLAY
          unset QT_QPA_PLATFORMTHEME QT_STYLE_OVERRIDE
          export QT_QPA_PLATFORM=offscreen

          exec ${gowinFhs}/bin/gowin-fhs -c "cd '$(pwd)' && unset DISPLAY QT_QPA_PLATFORMTHEME QT_STYLE_OVERRIDE && QT_QPA_PLATFORM=offscreen LD_LIBRARY_PATH='$WORKSPACE_DIR/IDE/lib':\"$LD_LIBRARY_PATH\" '$WORKSPACE_DIR/IDE/bin/gw_sh' $*"
        '';
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            # Gowin IDE (CLI synthesis)
            gwSh
            gowinEda

            # Open-source synthesis (linting only for GW5A)
            yosys

            # Programming tool
            openfpgaloader

            # Simulation & waveform
            verilator
            gtkwave

            # Build tools
            gnumake

            # FTDI EEPROM programming (FT2232H async 245 setup)
            libftdi1

            # Rust and WebAssembly toolchain for spi-flash-tool and the Web UI
            rustToolchain
            trunk
            wasmBindgenCli
            pkg-config
            udev
          ];

          shellHook = ''
            echo "Tang Primer 25K SPI Flash Emulator Development Environment"
            echo ""
            echo "Available tools:"
            echo "  gw_sh            - Gowin CLI synthesis (Education Edition)"
            echo "  gw_ide           - Gowin IDE GUI"
            echo "  yosys            - Verilog linting"
            echo "  openFPGALoader   - FPGA programming"
            echo "  verilator        - Verilog simulation"
            echo "  gtkwave          - Waveform viewer"
            echo "  ftdi_eeprom      - FT2232H EEPROM programmer"
            echo "  cargo            - Rust build tool (including wasm32 target support)"
            echo "  wasm-bindgen      - Generate browser bindings for the Web UI"
            echo "  trunk             - Build and serve the Web UI"
            echo ""
            echo "Build commands:"
            echo "  make build       - Synthesize with gw_sh (CLI)"
            echo "  make prog        - Program FPGA (volatile)"
            echo "  make flash       - Program to flash (persistent)"
            echo "  make lint        - Lint with yosys"
            echo "  make tool        - Build spi-flash-tool"
            echo "  make webui-serve - Build and serve the browser UI"
            echo ""
          '';
        };
      }
    );
}
