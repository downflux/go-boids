# go-boids
Golang implementation of Boid flocking

## Example

```bash
go run \
  github.com/downflux/go-boids/demo/generator | go run \
  github.com/downflux/go-boids/demo > demo.gif
```

![collision.gif](demo/output/collision.gif)

## Profiling

```
go test -v \
  github.com/downflux/go-boids/... \
  -benchmem \
  -cpuprofile cpu.out
  -memprofile mem.out

go tool pprof -tree -nodecount=10 cpu.out
```
