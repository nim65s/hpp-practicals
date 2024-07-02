{
  lib,
  cmake,
  hpp-gepetto-viewer,
  hpp-gui,
  hpp-plot,
  pkg-config,
  python3Packages,
  libsForQt5,
}:

python3Packages.buildPythonPackage {
  pname = "hpp-practicals";
  version = "5.0.0";
  pyproject = false;

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

  strictDeps = true;

  nativeBuildInputs = [
    cmake
    libsForQt5.wrapQtAppsHook
    pkg-config
  ];
  buildInputs = [ libsForQt5.qtbase ];
  propagatedBuildInputs = [
    hpp-gepetto-viewer
    hpp-gui
    hpp-plot
  ];

  doCheck = true;

  meta = {
    description = "Practicals for Humanoid Path Planner software";
    homepage = "https://github.com/humanoid-path-planner/hpp-practicals";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
