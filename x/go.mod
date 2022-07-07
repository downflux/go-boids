module github.com/downflux/go-boids/x

go 1.18

require (
	github.com/downflux/go-boids v0.0.0-20220702210039-50150811c0d3
	github.com/downflux/go-geometry v0.10.1
	github.com/downflux/go-kd v0.4.2
)

require github.com/downflux/go-orca v0.3.0 // indirect

replace github.com/downflux/go-boids => ../
