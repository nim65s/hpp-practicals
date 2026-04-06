{
  description = "Practicals for Humanoid Path Planner software";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    systems.follows = "gepetto/systems";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          {
            flakoboros.overrideAttrs.hpp-practicals = _: {
              src = lib.fileset.toSource {
                root = ./.;
                fileset = lib.fileset.unions [
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
            };
          }
        ];
      }
    );
}
