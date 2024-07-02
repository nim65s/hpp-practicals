{
  description = "Practicals for Humanoid Path Planner software";

  nixConfig = {
    extra-substituters = [ "https://gepetto.cachix.org" ];
    extra-trusted-public-keys = [ "gepetto.cachix.org-1:toswMl31VewC0jGkN6+gOelO2Yom0SOHzPwJMY2XiDY=" ];
  };

  inputs = {
    nixpkgs.url = "github:nim65s/nixpkgs/gepetto";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    hpp-gepetto-viewer = {
      url = "github:humanoid-path-planner/hpp-gepetto-viewer";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
        hpp-corbaserver.follows = "hpp-gui/hpp-manipulation-corba/hpp-corbaserver";
      };
    };
    hpp-gui = {
      url = "github:humanoid-path-planner/hpp-gui";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
      };
    };
    hpp-plot = {
      url = "github:humanoid-path-planner/hpp-plot";
      inputs = {
        nixpkgs.follows = "nixpkgs";
        flake-parts.follows = "flake-parts";
        hpp-manipulation-corba.follows = "hpp-gui/hpp-manipulation-corba";
      };
    };
  };

  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ ];
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        {
          self',
          pkgs,
          system,
          ...
        }:
        {
          packages = {
            inherit (pkgs) cachix;
            default = pkgs.callPackage ./. {
              hpp-gepetto-viewer = inputs.hpp-gepetto-viewer.packages.${system}.default;
              hpp-gui = inputs.hpp-gui.packages.${system}.default;
              hpp-plot = inputs.hpp-plot.packages.${system}.default;
            };
          };
          devShells.default = pkgs.mkShell { inputsFrom = [ self'.packages.default ]; };
        };
    };
}
