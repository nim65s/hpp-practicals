{
  description = "Practicals for Humanoid Path Planner software";

  inputs.gepetto.url = "github:gepetto/nix";

  outputs =
    inputs:
    inputs.gepetto.lib.mkFlakoboros inputs (
      { lib, ... }:
      {
        overrideAttrs.hpp-practicals = {
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
    );
}
