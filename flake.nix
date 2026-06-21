{
  description = "Tang Primer 25K SPI Flash Emulator";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    gowin-eda = {
      url = "github:Blue-Berry/gowin-eda.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
      gowin-eda,
      fenix,
    }:
    flake-utils.lib.eachSystem [ "x86_64-linux" ] (
      system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };

        # Gowin EDA Education edition from gowin-eda.nix
        gowinEda = gowin-eda.packages.${system}.default;

        # Rust toolchain for both the host spi-flash-tool and the RP2040 Pico
        # firmware.  The Pico firmware targets Cortex-M0+ and needs the
        # thumbv6m-none-eabi standard library available in the dev shell, not a
        # rustup-installed side effect.
        picoRustToolchain = fenix.packages.${system}.combine [
          fenix.packages.${system}.stable.cargo
          fenix.packages.${system}.stable.clippy
          fenix.packages.${system}.stable.rust-src
          fenix.packages.${system}.stable.rustc
          fenix.packages.${system}.stable.rustfmt
          fenix.packages.${system}.targets.thumbv6m-none-eabi.stable.rust-std
        ];

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
              udev
            ];
          runScript = "bash";
        };

        norbertUdevRules = pkgs.writeTextFile {
          name = "norbert-udev-rules";
          destination = "/etc/udev/rules.d/99-norbert.rules";
          text = ''
            # NORbert Pico USB bridge
            SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="4e42", MODE="0660", GROUP="plugdev", TAG+="uaccess"

            # Raspberry Pi Debug Probe (CMSIS-DAP)
            SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="000c", MODE="0660", GROUP="plugdev", TAG+="uaccess"

            # Tang Primer / FT2232H JTAG+FIFO adapter
            SUBSYSTEM=="usb", ATTR{idVendor}=="0403", ATTR{idProduct}=="6010", MODE="0660", GROUP="plugdev", TAG+="uaccess"
          '';
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

          unset QT_QPA_PLATFORMTHEME QT_STYLE_OVERRIDE

          # Gowin's "CLI" still initializes Qt/OpenGL.  Running it directly on
          # the pi/headless X display aborts with "Could not initialize GLX", so
          # give it a tiny software-rendered X server and force the xcb backend.
          export QT_QPA_PLATFORM=xcb
          export QT_XCB_GL_INTEGRATION=none
          export QT_OPENGL=software
          export LIBGL_ALWAYS_SOFTWARE=1
          export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

          quoted_args=""
          for arg in "$@"; do
            printf -v quoted_arg ' %q' "$arg"
            quoted_args+="$quoted_arg"
          done

          exec ${pkgs.xvfb-run}/bin/xvfb-run -a -s "-screen 0 1280x1024x24 +extension GLX +render" \
            ${gowinFhs}/bin/gowin-fhs -c "cd '$(pwd)' && env QT_QPA_PLATFORM=xcb QT_XCB_GL_INTEGRATION=none QT_OPENGL=software LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe LD_LIBRARY_PATH='$WORKSPACE_DIR/IDE/lib':\"\$LD_LIBRARY_PATH\" '$WORKSPACE_DIR/IDE/bin/gw_sh'$quoted_args"
        '';
      in
      {
        packages.udev-rules = norbertUdevRules;

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

            # Rust toolchain for spi-flash-tool and pico-firmware
            picoRustToolchain
            pkg-config
            probe-rs-tools
            systemd # udev
            udev
          ];

          # Cargo-built tools dynamically link libudev via rusb/serialport.
          # Keep the released binary runnable from inside nix develop.
          LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath [ pkgs.udev ];

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
            echo "  cargo            - Rust build tool"
            echo ""
            if [ -e /dev/bus/usb ]; then
              pico_usb_dev=$(for d in /sys/bus/usb/devices/*; do
                [ -f "$d/idVendor" ] || continue
                [ "$(cat "$d/idVendor")" = "1209" ] || continue
                [ "$(cat "$d/idProduct")" = "4e42" ] || continue
                printf '/dev/bus/usb/%03d/%03d\n' "$(cat "$d/busnum")" "$(cat "$d/devnum")"
              done | head -n1)
              if [ -n "$pico_usb_dev" ] && [ ! -w "$pico_usb_dev" ]; then
                echo "Warning: $pico_usb_dev is not writable; install .#udev-rules for --pico-usb access."
              fi
            fi
            echo "Build commands:"
            echo "  make build       - Synthesize with gw_sh (CLI)"
            echo "  make prog        - Program FPGA (volatile)"
            echo "  make flash       - Program to flash (persistent)"
            echo "  make lint        - Lint with yosys"
            echo "  make tool        - Build spi-flash-tool"
            echo ""
          '';
        };
      }
    )
    // {
      nixosModules.default =
        { pkgs, ... }:
        {
          services.udev.packages = [ self.packages.${pkgs.stdenv.hostPlatform.system}.udev-rules ];
        };
    };
}
