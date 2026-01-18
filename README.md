# Differential Game Simulator

A simple C simulator and renderer for working with differential games. Based on
SDL2.

## Building

To build the entire project, just run `make` in the project directory. This will
compile the entire library and all the example binaries.

## Examples

Included examples can be found in the `examples/` directory. To make a specific
example, you can use `make bin/<example>` where `<example>` is the name of the C
file without the `.c` suffix.

Ex: to build and run the `particle` example, you can use:
```console
$ make bin/particle
$ ./bin/particle
```

### Particle

A particle which follows your mouse forever.

![Particle](./docs/particle.png)

### 2 Pursuers 2 Evaders

An example of a 2 pursuer, 2 evader differential game.

![2P2E](./docs/2p2e.png)
