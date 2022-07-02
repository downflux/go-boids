package agent

import (
	"testing"

	"github.com/downflux/go-boids/agent"
	"github.com/downflux/go-geometry/2d/vector"
)

// TODO(minkezhang): Fill out the following test for the following scenarios --
//
//   Miss/RelativeStationary
//   Miss/Slide
//   Collide/Direct
//   Collide/Direct/NonTouching
func TestForce(t *testing.T) {
	configs := []struct {
		name     string
		obstacle agent.RO
		agent    agent.RO
		k        float64
		tau      float64
		want     vector.V
	}{}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := New(O{
				Obstacle: c.obstacle,
				K:        c.k,
			}).Force(c.agent); !vector.Within(got, c.want) {
				t.Errorf("A() = %v, want = %v", got, c.want)
			}
		})
	}
}
