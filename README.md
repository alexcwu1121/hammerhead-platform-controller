# hammerhead-platform-controller

To build:
1. Install Podman >4.x
2. Install VSCode
3. Install VSCode Dev Containers extension
4. In Dev Containers settings, set the following:
    - Docker Path: `podman`
    - Docker Compose Path: `podman-compose`
    - Docker Socket: `/run/podman/podman.sock`

To quick-run pre-commit, builds, tests:
`podman-compose run --build pre-commit`
`podman-compose run --build build-debug`
`podman-compose run --build build-release`

CAN ID claims:
- Pub: 0x100 - 0x1FF
- Sub: 0x200 - 0x2FF
