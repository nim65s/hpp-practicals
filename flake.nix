{
  description = "Practicals for Humanoid Path Planner software";

  inputs = {
    nixpkgs.url = "github:gepetto/nixpkgs";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    gerard-bauzil = {
      url = "github:gepetto/gerard-bauzil/devel";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
    };
    gtsp-laas = {
      url = "git+https://gitlab.laas.fr/gsaurel/gtsp-laas?ref=nix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
    };
    hpp-task-sequencing = {
      url = "github:nim65s/hpp-task-sequencing/nix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = inputs.nixpkgs.lib.systems.flakeExposed;
      perSystem =
        {
          pkgs,
          self',
          system,
          ...
        }:
        {
          devShells.default = pkgs.mkShell {
            packages = [
              (pkgs.python3.withPackages (_: [
                self'.packages.default
                inputs.hpp-task-sequencing.packages.${system}.hpp-task-sequencing
                inputs.gtsp-laas.packages.${system}.gtsp-laas
              ]))
            ];
            env.ROS_PACKAGE_PATH = pkgs.lib.concatStringsSep ":" (
              map (p: "${p}/share") [
                inputs.gerard-bauzil.packages.${system}.gerard-bauzil
                self'.packages.default
                self'.packages.pmb2-meshes
                self'.packages.tiago-data
                self'.packages.tiago-meshes
                self'.packages.hey5-meshes
              ]
            );
            shellHook = ''
              export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${
                pkgs.lib.makeLibraryPath [
                  pkgs.hpp-manipulation
                  pkgs.hpp-manipulation-corba
                ]
              }
            '';
          };
          packages = {
            default = self'.packages.hpp-practicals;
            hpp-practicals = pkgs.python3Packages.hpp-practicals.overrideAttrs (_: {
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
            hey5-meshes = pkgs.callPackage (
              { fetchFromGitHub, stdenvNoCC }:
              stdenvNoCC.mkDerivation rec {
                pname = "hey5-meshes";
                version = "3.0.4";
                src = fetchFromGitHub {
                  owner = "pal-robotics";
                  repo = "hey5_description";
                  rev = version;
                  hash = "sha256-xcfaeu5Gj/H2dDbkSsVVFwDDYXJzqsOGUAVQfm7ehyg=";
                };
                dontConfigure = true;
                dontBuild = true;
                installPhase = ''
                  mkdir -p $out/share/hey5_description
                  cp -r meshes $out/share/hey5_description
                '';
              }
            ) { };

            pmb2-meshes = pkgs.callPackage (
              { fetchFromGitHub, stdenvNoCC }:
              stdenvNoCC.mkDerivation rec {
                pname = "pmb2-meshes";
                version = "5.2.0";
                src = fetchFromGitHub {
                  owner = "pal-robotics";
                  repo = "pmb2_robot";
                  rev = version;
                  hash = "sha256-suHbl5eTIHGz5CLCkbyZU8U4NSFG0R7EaLKVBz8+igk=";
                };
                dontConfigure = true;
                dontBuild = true;
                installPhase = ''
                  mkdir -p $out/share/pmb2_description
                  cp -r pmb2_description/meshes $out/share/pmb2_description
                '';
              }
            ) { };
            tiago-data = pkgs.callPackage (
              { fetchFromGitLab, stdenvNoCC }:
              stdenvNoCC.mkDerivation rec {
                pname = "tiago-data";
                version = "1.1.0";
                src = fetchFromGitLab {
                  domain = "gitlab.laas.fr";
                  owner = "stack-of-tasks";
                  repo = "tiago_data";
                  rev = "v${version}";
                  hash = "sha256-i9vqWi6OR4HMePBnE3MB6cgmI3hDDmGQ7S9IAgLHQm4=";
                };
                dontConfigure = true;
                dontBuild = true;
                installPhase = ''
                  mkdir -p $out/share/tiago_data
                  cp -r gazebo meshes robots srdf urdf $out/share/tiago_data/
                '';
              }
            ) { };
            tiago-meshes = pkgs.callPackage (
              { fetchFromGitHub, stdenvNoCC }:
              stdenvNoCC.mkDerivation rec {
                pname = "tiago-meshes";
                version = "4.5.0";
                src = fetchFromGitHub {
                  owner = "pal-robotics";
                  repo = "tiago_robot";
                  rev = version;
                  hash = "sha256-sDnApNqJiSkxZxcYB6lGspM0t0KTOHq/MPs+ECLoDX0=";
                };
                dontConfigure = true;
                dontBuild = true;
                installPhase = ''
                  mkdir -p $out/share/tiago_description
                  cp -r tiago_description/meshes $out/share/tiago_description/
                '';
              }
            ) { };
          };
        };
    };
}
