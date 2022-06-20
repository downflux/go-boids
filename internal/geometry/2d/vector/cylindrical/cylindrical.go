package cylindrical

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type V vector.V

func (v V) R() float64     { return vector.V(v).X() }
func (v V) Theta() float64 { return vector.V(v).Y() }
