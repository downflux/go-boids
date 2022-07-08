package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"

	"github.com/downflux/go-boids/x/demo/config"
)

var (
	in   = flag.String("out", "/dev/stdout", "")
	mode = flag.String("mode", "grid", "")
)

func main() {
	flag.Parse()

	fp, err := os.OpenFile(*in, os.O_RDWR|os.O_CREATE, 0666)
	if err != nil {
		panic(err)
	}
	defer fp.Close()

	c, ok := map[string]config.C{
		"grid": config.GenerateGrid(10, 10),
	}[*mode]
	if !ok {
		panic(fmt.Sprintf("unsupported mode %v", *mode))
	}

	e := json.NewEncoder(fp)
	e.SetIndent("", "  ")

	_ = e.Encode(&c)
}
