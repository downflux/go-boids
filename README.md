# go-boids
Golang implementation of Boid flocking

## Example

```bash
go run \
  github.com/downflux/go-boids/demo/generator --mode=grid | go run \
  github.com/downflux/go-boids/demo --frames=1500 --log_dir=demo/output/log/ > collision.gif
```

![collision.gif](demo/output/collision.gif)

## Profiling

```bash
go test -v \
  github.com/downflux/go-boids/... \
  -benchmem \
  -cpuprofile cpu.out
  -memprofile mem.out

go tool pprof -tree -nodecount=10 cpu.out
```

## Sources

1. Parker, Conrad. "Boids Pseudocode."
   http://www.kfish.org/boids/pseudocode.html. 2023.
1. slsdo. https://slsdo.github.io/steering-behaviors/
1. Reynolds, Craig. "Flocks, Herds, and Schools: A Distributed Behavioral
   Model." https://team.inria.fr/imagine/files/2014/10/flocks-hers-and-schools.pdf. 1987.
1. Reynolds, Craig. "Steering Behaviors For Autonomous Characters."
   https://www.red3d.com/cwr/papers/1999/gdc99steer.pdf. 1999.
1. Saber, Reza. "Flocking with Obstacle Avoidance."
   https://authors.library.caltech.edu/28025/1/cdstr-ros-03-006.pdf. 2003.
1. Park et al. "Target Arrival Coordination in Boid Based Formation Flight."
   https://www.sciencedirect.com/science/article/pii/S1474667017323236. 2004.
