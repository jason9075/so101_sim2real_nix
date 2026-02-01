{
  description = "SO-101 Sim-to-Real Development Environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs; [
            docker-compose
            gnumake
            git
            xorg.xhost
            # Hardware communication helpers
            setserial
            python3
          ];

          shellHook = ''
            echo "Entering SO-101 Sim-to-Real environment"
            if [ ! -f .env ]; then
              echo "Warning: .env file not found. Please copy .env.example to .env and fill in the values."
            fi
          '';
        };
      }
    );
}
