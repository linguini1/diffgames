# Differential Game Simulator

A simple C simulator and renderer for working with differential games. Based on
SDL2.

## Building

### Linux

To build the entire project, just run `make` in the project directory. This will
compile the entire library and all the example binaries.

## Windows

You will need to install an SDL2 installation on your machine somewhere, and you
should be using the MinGW toolchain.

In order to build the examples, you should use the following command:
```console
$ make SDL_PATH=path/to/your/SDL2
```

In order to run any of the examples, ensure the `SDL2.dll` file that came in
your SDL2 installation's `bin/` directory is present in the directory where you
are running the example. Ex:

```console
$ cp path/to/your/SDL2/bin/SDL2.dll .
$ ./bin/particle.exe
```

## Examples

Included examples can be found in the `examples/` directory. To make a specific
example, you can use `make <example>` where `<example>` is the name of the
example's directory.

Ex: to build and run the `particle` example, you can use:
```console
$ make particle
$ ./bin/particle
```

**In all games, you can press 'q' or 'Esc' to quit the simulation.** For
specific information about the example you're using, run the example with the
`-h` flag:

```console
./bin/particle -h
```

### Particle

A particle which follows your mouse forever.

![Particle](./docs/particle.png)

### 2 Pursuers 2 Evaders

An example of a 2 pursuer, 2 evader differential game.

Press 'r' to toggle showing the capture radius of the pursuers.

![2P2E](./docs/2p2e.png)
