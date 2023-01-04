module github.com/downflux/go-boids/x

go 1.19

replace github.com/downflux/go-boids => ../

require (
	github.com/downflux/go-database v0.3.1
	github.com/downflux/go-geometry v0.15.4
)

require github.com/downflux/go-bvh v1.0.0 // indirect
