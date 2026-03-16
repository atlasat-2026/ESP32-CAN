{
  description = "A Nix-flake-based C/C++ development environment";
  inputs = {
    nixpkgs.url = "https://flakehub.com/f/NixOS/nixpkgs/0"; # stable Nixpkgs
    esp-dev.url = "github:mirrexagon/nixpkgs-esp-dev";
  };

  outputs = { self, ... }@inputs:

    let
      supportedSystems =
        [ "x86_64-linux" "aarch64-linux" "x86_64-darwin" "aarch64-darwin" ];
      forEachSupportedSystem = f:
        inputs.nixpkgs.lib.genAttrs supportedSystems (system:
          f {
            pkgs = import inputs.nixpkgs {
              inherit system;
              overlays = [ inputs.esp-dev.overlays.default ];
              config = {
                permittedInsecurePackages = [ "python3.13-ecdsa-0.19.1" ];
              };
            };
          });
    in {
      devShells = forEachSupportedSystem ({ pkgs }: {
        default = pkgs.mkShell.override { } {
          packages = with pkgs;
            [ esp-idf-full rustup openssl stdenv.cc.cc.lib libclang ]
            ++ (if system == "aarch64-darwin" then [ ] else [ gdb ]);
          shellHook = ''
            export CLANGD_QUERY_DRIVER="$(which clang),$(which clang++)"
            export CMAKE_EXPORT_COMPILE_COMMANDS=1
            export IDF_TOOLCHAIN="clang"
            export NVIM_ESP32_ENV=1

            export PATH="/home/flima/.rustup/toolchains/esp/xtensa-esp-elf/esp-15.2.0_20250920/xtensa-esp-elf/bin:$PATH"
            export LIBCLANG_PATH="/home/flima/.rustup/toolchains/esp/xtensa-esp32-elf-clang/esp-20.1.1_20250829/esp-clang/lib"

            # export LIBCLANG_PATH="${pkgs.libclang.lib}/lib"
            export LD_LIBRARY_PATH="${pkgs.stdenv.cc.cc.lib}/lib:$LD_LIBRARY_PATH"

          '';

        };
      });
    };
}
