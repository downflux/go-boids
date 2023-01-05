package separation

import (
	"testing"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/agent/mock"
	"github.com/downflux/go-database/flags"
	"github.com/downflux/go-geometry/2d/vector"
)

func TestSimpleAgentWithSingularity(t *testing.T) {
	type config struct {
		name string

		pSource vector.V
		vSource vector.V
		mSource float64
		rSource float64

		pObstacle vector.V
		vObstacle vector.V
		mObstacle float64
		rObstacle float64

		want vector.V
	}

	configs := []config{
		{
			name:      "Direct/Far",
			pSource:   vector.V{0, 0},
			vSource:   vector.V{1, 0},
			mSource:   1,
			rSource:   1,
			pObstacle: vector.V{10, 0},
			vObstacle: vector.V{-1, 0},
			mObstacle: 1,
			rObstacle: 1,
			want:      vector.V{-0.25, 0},
		},
		{
			name:      "Direct/Far/Light",
			pSource:   vector.V{0, 0},
			vSource:   vector.V{1, 0},
			mSource:   1,
			rSource:   1,
			pObstacle: vector.V{10, 0},
			vObstacle: vector.V{-1, 0},
			mObstacle: 10,
			rObstacle: 1,
			want:      vector.V{-2.5, 0},
		},
		{
			name:      "Direct/Near",
			pSource:   vector.V{0, 0},
			vSource:   vector.V{1, 0},
			mSource:   1,
			rSource:   1,
			pObstacle: vector.V{2.1, 0},
			vObstacle: vector.V{-1, 0},
			mObstacle: 1,
			rObstacle: 1,
			want:      vector.V{-20, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			source := mock.New(0, agent.O{
				Position: c.pSource,
				Velocity: c.vSource,
				Mass:     c.mSource,
				Radius:   c.rSource,
				Flags:    flags.FSizeSmall,
			})
			obstacle := mock.New(1, agent.O{
				Position: c.pObstacle,
				Velocity: c.vObstacle,
				Mass:     c.mObstacle,
				Radius:   c.rObstacle,
				Flags:    flags.FSizeSmall,
			})

			if got := SimpleAgentWithSingularity(source, obstacle); !vector.Within(got, c.want) {
				t.Errorf("SimpleAgentWithSingularity() = %v, want = %v", got, c.want)
			}
		})
	}
}
