Instructions for the BE
=======================

  . HPP is installed via bianry packages,
  . hpp-practicals is installed in /workspace
  . in each terminal, run bash to switch to bash.
  . to build the image:
    DOCKER_BUILDKIT=1 docker build -t be-hpp:2022 -f Dockerfile .
  . to run the image
    docker run --rm --name hpp -p 6080:6080 be-hpp:2022
