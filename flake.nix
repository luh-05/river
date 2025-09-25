{
  description = "RIvEr Sandbox";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs =
    {
      self,
      nixpkgs,
      flake-utils,
    }:
    flake-utils.lib.eachDefaultSystem (
      system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
        nativeBuildInputs = with pkgs; [
          zig
          pkg-config
          lazygit
        ];
      in
      {
        devShell = pkgs.mkShell {
          inherit nativeBuildInputs;
          shellHook = ''
            alias la="eza -lA"
            echo "RIvEr dev shell ready."
          '';
        };
      }
    );
}
