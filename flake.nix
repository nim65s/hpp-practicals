{
  description = "Practicals for Humanoid Path Planner software";

  inputs = {
    nixpkgs.url = "github:gepetto/nixpkgs";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        { pkgs, self', ...  }:
        {
          devShells.default = pkgs.mkShell {
            inputsFrom = [ self'.packages.default ];
            packages = [
              (pkgs.python3.withPackages (p: [
                p.hpp-gepetto-viewer
                p.hpp-manipulation-corba
              ]))
            ];
          };
          packages = {
            default = pkgs.python3Packages.hpp-practicals.overrideAttrs (_: {
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./docker
                  ./instructions
                  ./meshes
                  ./package.xml
                  ./script
                  ./slides
                  ./src
                  ./srdf
                  ./update
                  ./urdf
                ];
              };
            });
          };
        };
    };
}
